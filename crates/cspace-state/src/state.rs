// Copyright (c) 2013, Ioan A. Sucan
// Copyright (c) 2013, Willow Garage, Inc.
// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// Ported from moveit2 @ e017c91ee12984393a28ba246075c65f69cde3bf:
//   moveit_core/robot_state/include/moveit/robot_state/robot_state.hpp
//   moveit_core/robot_state/src/robot_state.cpp

use std::collections::HashMap;
use std::f64::consts::PI;
use std::ops::Deref;

use nalgebra::DMatrix;

use cspace_error::{Error, Result};
use cspace_geometry::{Isometry3, Vector3};
use cspace_model::joint::{JointKind, JointModel, PlanarJoint};
use cspace_model::{JointModelGroup, RobotModel};
use rand::{Rng, RngExt};
use rand_distr::StandardNormal;

use crate::numeric::{cxx_max, cxx_min};

/// An index into [`RobotModel::joint_models`](cspace_model::RobotModel::joint_models).
///
/// Upstream identifies a joint by `const JointModel*`; this port uses its
/// position in the model's joint array instead, matching the index
/// convention [`RobotModel`] itself already uses.
pub type JointIndex = usize;

/// Variables in the widest joint this port models — a floating joint's
/// `x y z qx qy qz qw`. Used to stage one joint's interpolation output
/// without allocating; a wider joint kind would have to grow this.
const MAX_JOINT_VARIABLES: usize = 7;

/// `checkInterpolationParamBounds`
/// (`robot_model.hpp:63`): NaN and infinity throw; a `t` outside `[0, 1]`
/// only warns, and extrapolates. The warning is dropped rather than routed
/// somewhere — this port has no logger, and turning upstream's warning into
/// an error would reject the extrapolation upstream performs.
fn check_interpolation_param_bounds(t: f64) -> Result<()> {
    if t.is_nan() || t.is_infinite() {
        return Err(Error::other("Interpolation parameter is NaN or inf."));
    }
    Ok(())
}

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
///   lockstep, and this port's [`cspace_model::LinkModel`] carries no
///   collision geometry yet (deferred to a later `cspace-collision` phase —
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
///    `effort_or_acceleration_` vector (`robot_state.hpp:1730`); this port
///    gives each its own `Vec<f64>`, so a value written to one is not
///    overwritten by a later write to the other.
///
///    The *exclusivity* that aliasing enforces is not a deviation, and is
///    reproduced here, by the private `Dynamics` sum type this port
///    switches on in place of the two bools. It is a documented public
///    guarantee, not a side effect of the memory layout:
///    `robot_state.hpp:320` and `:418` both state that when one of
///    `hasAccelerations()`/`hasEffort()` reports true the other "will
///    certainly report false", and name serialization and state copying as
///    what relies on it.
///
///    What remains deviating is the *value* upstream leaves behind on a
///    transition: `markAcceleration`/`markEffort` (`robot_state.cpp:175`,
///    `:185`) zero the shared buffer when the marked quantity was not
///    already the live one, so upstream's acceleration reads back as `0.0`
///    at every variable a partial write did not touch. With separate
///    buffers this port has no stale sibling data to erase, and leaves the
///    previous acceleration values in place instead.
///
///    [`RobotState::enforce_bounds`]/[`RobotState::satisfies_bounds`]
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
///    [`cspace_model::RobotModel`] deliberately does not build that table
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
    /// storage instead (see this type's doc comment). Readable only when
    /// [`RobotState::has_accelerations`] is true.
    acceleration: Vec<f64>,
    /// See `acceleration`'s doc comment.
    effort: Vec<f64>,
    has_velocity: bool,
    /// Which of `acceleration`/`effort` this state carries, replacing
    /// upstream's `has_acceleration_`/`has_effort_` pair.
    dynamics: Dynamics,
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

/// Which dynamics quantity a [`RobotState`] carries, replacing upstream's
/// `has_acceleration_`/`has_effort_` pair with the one field they encode.
///
/// Upstream keeps the two bools mutually exclusive by hand:
/// `markAcceleration()` sets `has_acceleration_` and clears `has_effort_`,
/// `markEffort()` does the reverse (`robot_state.cpp:175-193`), and the
/// bulk `setVariableAccelerations(const double*)`/`setVariableEffort(const
/// double*)` overloads write both flags inline (`robot_state.hpp:350-351`,
/// `:447-448`). Every write site therefore has to remember to clear its
/// sibling; this port cannot forget, because there is no sibling to clear.
///
/// The exclusivity is load-bearing, not incidental to upstream's aliased
/// buffer: `robot_state.hpp:320`/`:418` promise callers that if one of
/// `hasAccelerations()`/`hasEffort()` reports true the other "will
/// certainly report false", and cite serializing and copying the state as
/// the reason to care. `RobotTrajectory`'s waypoint dump
/// (`robot_trajectory.cpp:679,687`) and every `JointTrajectoryPoint` built
/// from a state read exactly those two predicates, so a state with both
/// true emits a message upstream cannot produce.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Dynamics {
    /// Neither has been set — a freshly constructed state, matching
    /// upstream's `has_acceleration_(false), has_effort_(false)`
    /// (`robot_state.cpp:69-70`).
    None,
    /// `has_acceleration_ == true`.
    Acceleration,
    /// `has_effort_ == true`.
    Effort,
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
            dynamics: Dynamics::None,
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
    // why this port gives them independent storage instead, and `Dynamics`
    // for why it still reproduces the two flags' exclusivity. None of these
    // setters propagate to mimic joints — upstream's own
    // `setVariableVelocity`/`setVariableAcceleration`/`setVariableEffort`
    // do not call `updateMimicJoint` either.

    /// `hasVelocities`
    pub fn has_velocities(&self) -> bool {
        self.has_velocity
    }

    /// `hasAccelerations`
    ///
    /// Upstream's own contract (`robot_state.hpp:320`): when this reports
    /// true, [`RobotState::has_effort`] "will certainly report false".
    pub fn has_accelerations(&self) -> bool {
        self.dynamics == Dynamics::Acceleration
    }

    /// `hasEffort`
    ///
    /// Upstream's own contract (`robot_state.hpp:418`): when this reports
    /// true, [`RobotState::has_accelerations`] "will certainly report
    /// false".
    pub fn has_effort(&self) -> bool {
        self.dynamics == Dynamics::Effort
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

    /// `setVariableVelocities(const double*)`/`setVariableVelocities(const
    /// std::vector<double>&)`: replace every velocity at once.
    ///
    /// # Panics
    ///
    /// If `values.len()` is less than
    /// [`RobotModel::variable_count`](cspace_model::RobotModel::variable_count).
    /// Upstream's `std::vector` overload requires only `variable_count <=
    /// velocity.size()`, enforced by a debug-only
    /// `assert(getVariableCount() <= velocity.size())` (`robot_state.hpp`) —
    /// a `values` *longer* than needed is accepted silently there, since
    /// its own `double*` primitive `memcpy`s only the first
    /// `variable_count` entries and never reads the rest. This port matches
    /// that truncation exactly (`&values[..variable_count]`); it does not
    /// match upstream's debug-only rejection of a *shorter* `values`
    /// followed by a release-mode out-of-bounds `memcpy` read — Rust has no
    /// safe equivalent of that unchecked read, so a short `values` panics
    /// here deterministically, in every build profile, instead.
    pub fn set_variable_velocities(&mut self, values: &[f64]) {
        let len = self.velocity.len();
        self.velocity.copy_from_slice(&values[..len]);
        self.has_velocity = true;
    }

    /// `setVariableAccelerations(const double*)`/`setVariableAccelerations(const
    /// std::vector<double>&)`
    ///
    /// Clears [`RobotState::has_effort`], as upstream's own body does
    /// (`robot_state.hpp:350-351` writes both flags inline rather than
    /// going through `markAcceleration()`).
    ///
    /// # Panics
    ///
    /// See [`RobotState::set_variable_velocities`].
    pub fn set_variable_accelerations(&mut self, values: &[f64]) {
        let len = self.acceleration.len();
        self.acceleration.copy_from_slice(&values[..len]);
        self.dynamics = Dynamics::Acceleration;
    }

    /// `setVariableEffort(const double*)`/`setVariableEffort(const
    /// std::vector<double>&)`: replace every effort value at once. Named
    /// `_efforts` (upstream overloads on parameter type, which Rust cannot)
    /// to stay distinct from the per-variable
    /// [`RobotState::set_variable_effort`].
    ///
    /// Clears [`RobotState::has_accelerations`], as upstream's own body
    /// does (`robot_state.hpp:447-448` writes both flags inline rather than
    /// going through `markEffort()`).
    ///
    /// # Panics
    ///
    /// See [`RobotState::set_variable_velocities`].
    pub fn set_variable_efforts(&mut self, values: &[f64]) {
        let len = self.effort.len();
        self.effort.copy_from_slice(&values[..len]);
        self.dynamics = Dynamics::Effort;
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

    /// `setVariableVelocities(const std::map<std::string, double>&)`
    ///
    /// # Errors
    ///
    /// [`Error::UnknownName`] if any key is not a variable in this model.
    /// Upstream throws partway through the map on the first unknown key,
    /// leaving earlier entries already applied; this port does the same —
    /// see [`RobotState::set_variable_positions_by_name`]'s doc comment for
    /// why the differing iteration order is immaterial.
    pub fn set_variable_velocities_by_name(&mut self, values: &HashMap<String, f64>) -> Result<()> {
        for (name, &value) in values {
            self.set_variable_velocity(name, value)?;
        }
        Ok(())
    }

    /// `setVariableVelocities(const std::map<std::string, double>&,
    /// std::vector<std::string>&)`: as
    /// [`RobotState::set_variable_velocities_by_name`], plus every model
    /// variable absent from `values` (see [`RobotState::missing_keys`]).
    ///
    /// # Errors
    ///
    /// [`Error::UnknownName`] if any key is not a variable in this model.
    pub fn set_variable_velocities_by_name_and_missing(
        &mut self,
        values: &HashMap<String, f64>,
    ) -> Result<Vec<String>> {
        self.set_variable_velocities_by_name(values)?;
        Ok(self.missing_keys(values))
    }

    /// `setVariableVelocities(const std::vector<std::string>&, const
    /// std::vector<double>&)`
    ///
    /// # Errors
    ///
    /// [`Error::UnknownName`] if any name is not a variable in this model.
    pub fn set_variable_velocities_named(&mut self, names: &[&str], values: &[f64]) -> Result<()> {
        debug_assert_eq!(names.len(), values.len());
        for (&name, &value) in names.iter().zip(values) {
            self.set_variable_velocity(name, value)?;
        }
        Ok(())
    }

    /// `setJointVelocities(const JointModel*, const double*)`: one joint's
    /// own variables. Unlike [`RobotState::set_joint_positions`], upstream's
    /// `setJointVelocities` does not mark anything dirty and does not
    /// propagate to mimic joints — it only writes `velocity_` and
    /// `has_velocity_`, so this port matches that (no
    /// `RobotState::mark_dirty` / `RobotState::update_mimic_joint`
    /// calls here).
    ///
    /// # Errors
    ///
    /// [`Error::UnknownName`] if no joint is named `name`.
    ///
    /// # Panics
    ///
    /// If `values.len()` does not equal the joint's own variable count.
    pub fn set_joint_velocities(&mut self, name: &str, values: &[f64]) -> Result<()> {
        let joint_index = self.joint_index(name)?;
        let joint = self.model.joint_model_at(joint_index);
        if joint.variable_count() == 0 {
            return Ok(());
        }
        let first = self.first_variable_index[joint_index];
        self.velocity[first..first + joint.variable_count()].copy_from_slice(values);
        self.has_velocity = true;
        Ok(())
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
    /// Clears [`RobotState::has_effort`]: upstream reaches this write
    /// through `markAcceleration()` (`robot_state.hpp:389-393`).
    ///
    /// # Errors
    ///
    /// [`Error::UnknownName`] if `name` is not a variable in this model.
    pub fn set_variable_acceleration(&mut self, name: &str, value: f64) -> Result<()> {
        let index = self.model.variable_index(name)?;
        self.acceleration[index] = value;
        self.dynamics = Dynamics::Acceleration;
        Ok(())
    }

    /// `setVariableAcceleration(int, double)`
    ///
    /// Clears [`RobotState::has_effort`]; see
    /// [`RobotState::set_variable_acceleration`].
    pub fn set_variable_acceleration_at(&mut self, index: usize, value: f64) {
        self.acceleration[index] = value;
        self.dynamics = Dynamics::Acceleration;
    }

    /// `setVariableAccelerations(const std::map<std::string, double>&)`
    ///
    /// # Errors
    ///
    /// [`Error::UnknownName`] if any key is not a variable in this model.
    /// See [`RobotState::set_variable_velocities_by_name`]'s doc comment for
    /// the error-partway-through-the-map behavior this matches.
    pub fn set_variable_accelerations_by_name(
        &mut self,
        values: &HashMap<String, f64>,
    ) -> Result<()> {
        for (name, &value) in values {
            self.set_variable_acceleration(name, value)?;
        }
        Ok(())
    }

    /// `setVariableAccelerations(const std::map<std::string, double>&,
    /// std::vector<std::string>&)`: as
    /// [`RobotState::set_variable_accelerations_by_name`], plus every model
    /// variable absent from `values` (see [`RobotState::missing_keys`]).
    ///
    /// # Errors
    ///
    /// [`Error::UnknownName`] if any key is not a variable in this model.
    pub fn set_variable_accelerations_by_name_and_missing(
        &mut self,
        values: &HashMap<String, f64>,
    ) -> Result<Vec<String>> {
        self.set_variable_accelerations_by_name(values)?;
        Ok(self.missing_keys(values))
    }

    /// `setVariableAccelerations(const std::vector<std::string>&, const
    /// std::vector<double>&)`
    ///
    /// # Errors
    ///
    /// [`Error::UnknownName`] if any name is not a variable in this model.
    pub fn set_variable_accelerations_named(
        &mut self,
        names: &[&str],
        values: &[f64],
    ) -> Result<()> {
        debug_assert_eq!(names.len(), values.len());
        for (&name, &value) in names.iter().zip(values) {
            self.set_variable_acceleration(name, value)?;
        }
        Ok(())
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
    /// Clears [`RobotState::has_accelerations`]: upstream reaches this
    /// write through `markEffort()` (`robot_state.hpp:480-484`).
    ///
    /// # Errors
    ///
    /// [`Error::UnknownName`] if `name` is not a variable in this model.
    pub fn set_variable_effort(&mut self, name: &str, value: f64) -> Result<()> {
        let index = self.model.variable_index(name)?;
        self.effort[index] = value;
        self.dynamics = Dynamics::Effort;
        Ok(())
    }

    /// `setVariableEffort(int, double)`
    ///
    /// Clears [`RobotState::has_accelerations`]; see
    /// [`RobotState::set_variable_effort`].
    pub fn set_variable_effort_at(&mut self, index: usize, value: f64) {
        self.effort[index] = value;
        self.dynamics = Dynamics::Effort;
    }

    /// `setVariableEffort(const std::map<std::string, double>&)`. Named
    /// `_efforts_` (matching [`RobotState::set_variable_efforts`]) to stay
    /// distinct from the per-variable
    /// [`RobotState::set_variable_effort`]/[`RobotState::set_variable_effort_at`],
    /// the same reason [`RobotState::set_variable_efforts`]'s own doc
    /// comment gives.
    ///
    /// # Errors
    ///
    /// [`Error::UnknownName`] if any key is not a variable in this model.
    /// See [`RobotState::set_variable_velocities_by_name`]'s doc comment for
    /// the error-partway-through-the-map behavior this matches.
    pub fn set_variable_efforts_by_name(&mut self, values: &HashMap<String, f64>) -> Result<()> {
        for (name, &value) in values {
            self.set_variable_effort(name, value)?;
        }
        Ok(())
    }

    /// `setVariableEffort(const std::map<std::string, double>&,
    /// std::vector<std::string>&)`: as
    /// [`RobotState::set_variable_efforts_by_name`], plus every model
    /// variable absent from `values` (see [`RobotState::missing_keys`]).
    ///
    /// # Errors
    ///
    /// [`Error::UnknownName`] if any key is not a variable in this model.
    pub fn set_variable_efforts_by_name_and_missing(
        &mut self,
        values: &HashMap<String, f64>,
    ) -> Result<Vec<String>> {
        self.set_variable_efforts_by_name(values)?;
        Ok(self.missing_keys(values))
    }

    /// `setVariableEffort(const std::vector<std::string>&, const
    /// std::vector<double>&)`
    ///
    /// # Errors
    ///
    /// [`Error::UnknownName`] if any name is not a variable in this model.
    pub fn set_variable_efforts_named(&mut self, names: &[&str], values: &[f64]) -> Result<()> {
        debug_assert_eq!(names.len(), values.len());
        for (&name, &value) in names.iter().zip(values) {
            self.set_variable_effort(name, value)?;
        }
        Ok(())
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
    /// If `positions.len()` is less than
    /// [`RobotModel::variable_count`](cspace_model::RobotModel::variable_count).
    /// See [`RobotState::set_variable_velocities`]'s `# Panics` for why:
    /// upstream's precondition here is the identical `variable_count <=
    /// position.size()` pattern (`assert(getVariableCount() <=
    /// position.size())`, `robot_state.hpp`), tolerant of a longer
    /// `positions` and unchecked in release even for a shorter one.
    pub fn set_variable_positions(&mut self, positions: &[f64]) {
        let len = self.positions.len();
        self.positions.copy_from_slice(&positions[..len]);
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

    /// `getMissingKeys`: every model variable name absent from
    /// `variable_map`, excluding variables whose owning joint mimics
    /// another — a mimic's value is derived from its master rather than
    /// independently settable, so upstream does not count it as missing.
    /// Returned in [`RobotModel::variable_names`] order, matching upstream's
    /// iteration order over the same list.
    pub fn missing_keys(&self, variable_map: &HashMap<String, f64>) -> Vec<String> {
        self.model
            .variable_names()
            .iter()
            .enumerate()
            .filter(|(_, name)| !variable_map.contains_key(name.as_str()))
            .filter(|&(index, _)| {
                let joint_index = self.joint_of_variable[index];
                self.model.joint_model_at(joint_index).mimic().is_none()
            })
            .map(|(_, name)| name.clone())
            .collect()
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

    // ---- Group positions, velocities, accelerations --------------------
    //
    // Upstream declares each of these three times over (`const double*`,
    // `const std::vector<double>&`, `const Eigen::VectorXd&` — plus a
    // fourth `const std::string& joint_group_name` convenience layer that
    // just resolves the name and forwards). All three value-carrying forms
    // are the same pointer-plus-length pairing in different clothes, so
    // this port takes one `&[f64]` per setter, matching how
    // [`RobotState::set_variable_positions`]/[`RobotState::set_joint_positions`]
    // already collapse upstream's `double*`/`std::vector<double>&`
    // overloads above. Every group lookup goes through
    // [`RobotModel::joint_model_group`], matching this file's existing
    // `_group` variants ([`RobotState::enforce_bounds_group`],
    // [`RobotState::harmonize_positions_group`]) rather than upstream's
    // silent no-op on an unknown group name.

    /// `setJointGroupPositions(const JointModelGroup*, const double*)`: one
    /// value per variable in `group`'s own [`JointModelGroup::joint_indices`]
    /// order — *including* mimic joints' slots, whose supplied values are
    /// immediately overwritten by the mimic propagation below (matching
    /// upstream's own doc comment: "including values of mimic joints").
    ///
    /// # Errors
    ///
    /// [`Error::UnknownName`] if no group is named `group_name`.
    ///
    /// # Panics
    ///
    /// If `values` holds fewer entries than `group`'s variable count.
    /// Upstream's primitive — the `double*` overload every other overload
    /// in this family forwards to — performs **no** length check at all,
    /// not even a debug-only `assert`; a short buffer is undefined
    /// behaviour there. Rust has no safe equivalent of that unchecked read,
    /// so this port's closest faithful match is the slice index's own
    /// panic. A `values` slice *longer* than needed is accepted silently
    /// either way, matching upstream exactly — the trailing entries are
    /// never read.
    pub fn set_joint_group_positions(&mut self, group_name: &str, values: &[f64]) -> Result<()> {
        let model = self.model;
        let group = model.joint_model_group(group_name)?;
        let mut i = 0;
        for &joint_index in group.joint_indices() {
            let joint = model.joint_model_at(joint_index);
            let count = joint.variable_count();
            let first = self.first_variable_index[joint_index];
            self.positions[first..first + count].copy_from_slice(&values[i..i + count]);
            i += count;
        }
        self.update_mimic_joints_for_group(group);
        Ok(())
    }

    /// `setJointGroupActivePositions`: one value per variable in `group`'s
    /// own [`JointModelGroup::active_joint_indices`] order — *excluding*
    /// mimic joints, unlike [`RobotState::set_joint_group_positions`].
    ///
    /// Upstream writes each active joint through
    /// `setJointPositions(JointModel*, double*)`
    /// (`robot_state.cpp:600`), which — unlike this family's `Positions`
    /// primitive above — propagates each write to that joint's own mimic
    /// followers immediately (`updateMimicJoint`, global, not
    /// group-scoped) before the trailing group-wide
    /// `updateMimicJoints(group)` runs. A follower outside `group` that
    /// mimics an active joint written here is therefore updated too, same
    /// as upstream; that is not reachable by only calling
    /// `RobotState::update_mimic_joints_for_group`, which is why this
    /// setter also calls `RobotState::update_mimic_joint` per active
    /// joint, matching upstream's two-layer propagation exactly.
    ///
    /// # Errors
    ///
    /// [`Error::UnknownName`] if no group is named `group_name`.
    ///
    /// # Panics
    ///
    /// If `values` holds fewer entries than `group`'s *active* variable
    /// count. Upstream's own primitive here (unlike the `Positions`
    /// primitive above) does carry a debug-only `assert(gstate.size() ==
    /// group->getActiveVariableCount())`, compiled out entirely in a
    /// release build — so, same as
    /// [`RobotState::set_variable_positions`], this port enforces the
    /// bound unconditionally via the slice index rather than reproducing
    /// upstream's build-mode-dependent behaviour.
    pub fn set_joint_group_active_positions(
        &mut self,
        group_name: &str,
        values: &[f64],
    ) -> Result<()> {
        let model = self.model;
        let group = model.joint_model_group(group_name)?;
        let mut i = 0;
        for &joint_index in group.active_joint_indices() {
            let joint = model.joint_model_at(joint_index);
            let count = joint.variable_count();
            let first = self.first_variable_index[joint_index];
            self.positions[first..first + count].copy_from_slice(&values[i..i + count]);
            self.mark_dirty(joint_index);
            self.update_mimic_joint(joint_index);
            i += count;
        }
        self.update_mimic_joints_for_group(group);
        Ok(())
    }

    /// `copyJointGroupPositions`: `group`'s own variables, in
    /// [`JointModelGroup::joint_indices`] order (including mimic joints,
    /// matching [`RobotState::set_joint_group_positions`]'s own input
    /// order). Upstream copies rather than returning a pointer because a
    /// group's variables are "not necessarily a contiguous block of memory
    /// in the `RobotState` itself" (upstream's own doc comment); this port
    /// returns an owned `Vec` for the same reason, covering upstream's
    /// `double*`/`std::vector<double>&`/`Eigen::VectorXd&` out-param trio
    /// in one signature.
    ///
    /// # Errors
    ///
    /// [`Error::UnknownName`] if no group is named `group_name`.
    pub fn joint_group_positions(&self, group_name: &str) -> Result<Vec<f64>> {
        let group = self.model.joint_model_group(group_name)?;
        let mut out = Vec::with_capacity(group.variable_names().len());
        for &joint_index in group.joint_indices() {
            let joint = self.model.joint_model_at(joint_index);
            let count = joint.variable_count();
            let first = self.first_variable_index[joint_index];
            out.extend_from_slice(&self.positions[first..first + count]);
        }
        Ok(out)
    }

    /// `RobotState::updateMimicJoints(const JointModelGroup*)`: every mimic
    /// joint in `group`, derived from its master's *current* value —
    /// plus the group-dirty half of upstream's private
    /// `markDirtyJointTransforms(const JointModelGroup*)`
    /// (`robot_state.hpp:1686`), which marks every one of `group`'s
    /// *active* joints and merges in `group->getCommonRoot()`. This port
    /// caches no per-group common root (see [`JointModelGroup`]'s own doc
    /// comment on why), so the merge is done by folding
    /// [`RobotState::mark_dirty`] over the group's active joints instead —
    /// the lowest common ancestor of a node set is the same regardless of
    /// the order it is folded in, so the two are equivalent.
    ///
    /// Shared by [`RobotState::set_joint_group_positions`] and
    /// [`RobotState::set_joint_group_active_positions`], upstream's only
    /// two group setters that call `updateMimicJoints(group)`.
    fn update_mimic_joints_for_group(&mut self, group: &JointModelGroup) {
        for &mimic_index in group.mimic_joint_indices() {
            self.write_mimic(mimic_index);
        }
        for &joint_index in group.active_joint_indices() {
            self.mark_dirty(joint_index);
        }
    }

    /// `setJointGroupVelocities(const JointModelGroup*, const double*)`:
    /// one value per variable in `group`'s own
    /// [`JointModelGroup::joint_indices`] order, including mimic joints'
    /// own slots — unlike position, upstream never derives a mimic
    /// joint's *velocity* from its master (`setJointGroupVelocities` calls
    /// neither `updateMimicJoint` nor `updateMimicJoints`), so a mimic
    /// joint's slot here takes exactly the value `values` supplies, with
    /// no override.
    ///
    /// Does not call upstream's `markVelocity()` zero-fill step: that step
    /// only matters the first time velocity is ever set, to give the rest
    /// of the buffer a defined `0.0` rather than leftover memory: this
    /// port's `velocity` buffer is `vec![0.0; ...]` from
    /// [`RobotState::new`] and every mutator only ever writes finite
    /// values into it, so every not-yet-written slot is already `0.0` —
    /// matching how [`RobotState::set_variable_velocity`] and its
    /// siblings already only set `RobotState::has_velocity`, with no
    /// zero-fill of their own.
    ///
    /// # Errors
    ///
    /// [`Error::UnknownName`] if no group is named `group_name`.
    ///
    /// # Panics
    ///
    /// If `values` holds fewer entries than `group`'s variable count —
    /// same unchecked-primitive reasoning as
    /// [`RobotState::set_joint_group_positions`]: upstream's `double*`
    /// overload performs no length check at all.
    pub fn set_joint_group_velocities(&mut self, group_name: &str, values: &[f64]) -> Result<()> {
        let model = self.model;
        let group = model.joint_model_group(group_name)?;
        self.has_velocity = true;
        let mut i = 0;
        for &joint_index in group.joint_indices() {
            let joint = model.joint_model_at(joint_index);
            let count = joint.variable_count();
            let first = self.first_variable_index[joint_index];
            self.velocity[first..first + count].copy_from_slice(&values[i..i + count]);
            i += count;
        }
        Ok(())
    }

    /// `copyJointGroupVelocities`: `group`'s own velocities, in
    /// [`JointModelGroup::joint_indices`] order — reads
    /// `RobotState::velocity` regardless of
    /// [`RobotState::has_velocities`], matching upstream (which never
    /// checks `has_velocity_` in `copyJointGroupVelocities` either).
    ///
    /// # Errors
    ///
    /// [`Error::UnknownName`] if no group is named `group_name`.
    pub fn joint_group_velocities(&self, group_name: &str) -> Result<Vec<f64>> {
        let group = self.model.joint_model_group(group_name)?;
        let mut out = Vec::with_capacity(group.variable_names().len());
        for &joint_index in group.joint_indices() {
            let joint = self.model.joint_model_at(joint_index);
            let count = joint.variable_count();
            let first = self.first_variable_index[joint_index];
            out.extend_from_slice(&self.velocity[first..first + count]);
        }
        Ok(out)
    }

    /// `setJointGroupAccelerations(const JointModelGroup*, const double*)`:
    /// one value per variable in `group`'s own
    /// [`JointModelGroup::joint_indices`] order, including mimic joints'
    /// own slots — same "no mimic derivation for a dynamics quantity"
    /// rule as [`RobotState::set_joint_group_velocities`].
    ///
    /// Clears [`RobotState::has_effort`]: upstream reaches this write
    /// through `markAcceleration()` too (`robot_state.cpp:685-687`).
    ///
    /// Upstream's `markAcceleration()` additionally zeroes the shared
    /// buffer on the transition, so upstream leaves `0.0` at every
    /// variable outside `group`; this port's separate acceleration buffer
    /// keeps whatever was there. That value-level difference is the whole
    /// of "Deviations from upstream" §1 that survives — the flag
    /// exclusivity itself is reproduced.
    ///
    /// # Errors
    ///
    /// [`Error::UnknownName`] if no group is named `group_name`.
    ///
    /// # Panics
    ///
    /// If `values` holds fewer entries than `group`'s variable count —
    /// same unchecked-primitive reasoning as
    /// [`RobotState::set_joint_group_positions`]: upstream's `double*`
    /// overload performs no length check at all.
    pub fn set_joint_group_accelerations(
        &mut self,
        group_name: &str,
        values: &[f64],
    ) -> Result<()> {
        let model = self.model;
        let group = model.joint_model_group(group_name)?;
        self.dynamics = Dynamics::Acceleration;
        let mut i = 0;
        for &joint_index in group.joint_indices() {
            let joint = model.joint_model_at(joint_index);
            let count = joint.variable_count();
            let first = self.first_variable_index[joint_index];
            self.acceleration[first..first + count].copy_from_slice(&values[i..i + count]);
            i += count;
        }
        Ok(())
    }

    /// `copyJointGroupAccelerations`: `group`'s own accelerations, in
    /// [`JointModelGroup::joint_indices`] order — reads
    /// `RobotState::acceleration` regardless of
    /// [`RobotState::has_accelerations`], matching upstream.
    ///
    /// # Errors
    ///
    /// [`Error::UnknownName`] if no group is named `group_name`.
    pub fn joint_group_accelerations(&self, group_name: &str) -> Result<Vec<f64>> {
        let group = self.model.joint_model_group(group_name)?;
        let mut out = Vec::with_capacity(group.variable_names().len());
        for &joint_index in group.joint_indices() {
            let joint = self.model.joint_model_at(joint_index);
            let count = joint.variable_count();
            let first = self.first_variable_index[joint_index];
            out.extend_from_slice(&self.acceleration[first..first + count]);
        }
        Ok(out)
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

    /// `setToDefaultValues(group, name)`: `group`'s SRDF `<group_state>`
    /// named `name`, via [`JointModelGroup::variable_default_positions`].
    /// Returns whether such a state was found.
    ///
    /// Upstream builds a `std::map` from
    /// `JointModelGroup::getVariableDefaultPositions` and always forwards it
    /// to `setVariablePositions(map)`, even when the lookup failed and the
    /// map is empty — an empty map makes that call a no-op, so this port
    /// short-circuits instead of building and applying an empty map; the
    /// observable behavior (state unchanged, `false` returned) is the same.
    ///
    /// # Errors
    ///
    /// [`Error::UnknownName`] if no group is named `group_name`.
    pub fn set_to_default_values_group(&mut self, group_name: &str, name: &str) -> Result<bool> {
        let group = self.model.joint_model_group(group_name)?;
        let Some(defaults) = group.variable_default_positions(name) else {
            return Ok(false);
        };
        for (variable_name, &value) in defaults {
            self.set_variable_position(variable_name, value)?;
        }
        Ok(true)
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

    /// `setToRandomPositionsNearBy(group, seed, distance, rng)`: every
    /// active joint in `group` sampled near its value in `seed`, all with
    /// the same `distance`, via `sample_random_positions_near_by`.
    ///
    /// Upstream exposes this both with and without an explicit RNG
    /// parameter; this port only ever takes one explicit RNG, matching
    /// [`RobotState::set_to_random_positions_with`]'s own deviation (see its
    /// doc comment).
    ///
    /// # Errors
    ///
    /// [`Error::UnknownName`] if no group is named `group_name`.
    pub fn set_to_random_positions_near_by_group(
        &mut self,
        group_name: &str,
        seed: &Self,
        distance: f64,
        rng: &mut impl Rng,
    ) -> Result<()> {
        let model = self.model;
        let group = model.joint_model_group(group_name)?;
        for &joint_index in group.active_joint_indices() {
            let joint = model.joint_model_at(joint_index);
            let first = self.first_variable_index[joint_index];
            let count = joint.variable_count();
            sample_random_positions_near_by(
                joint,
                rng,
                &seed.positions[first..first + count],
                distance,
                &mut self.positions[first..first + count],
            );
        }
        self.update_mimic_joints_group(group);
        Ok(())
    }

    /// `setToRandomPositionsNearBy(group, seed, distances, rng)`: as
    /// [`RobotState::set_to_random_positions_near_by_group`], but with a
    /// per-joint distance instead of one shared by every joint.
    ///
    /// # Errors
    ///
    /// [`Error::UnknownName`] if no group is named `group_name`.
    ///
    /// # Panics
    ///
    /// If `distances.len()` is less than `group_name`'s active joint count.
    /// Upstream reads this unchecked past a debug-only `assert(distances.size()
    /// == joints.size())` — indexing `distances[i]` here fails the same way
    /// a release build's out-of-bounds read would, rather than silently
    /// truncating (which a `zip` over the two slices would do instead). A
    /// `distances` longer than the active joint count is not an error,
    /// matching upstream: the extra entries are simply never read.
    pub fn set_to_random_positions_near_by_group_with_distances(
        &mut self,
        group_name: &str,
        seed: &Self,
        distances: &[f64],
        rng: &mut impl Rng,
    ) -> Result<()> {
        let model = self.model;
        let group = model.joint_model_group(group_name)?;
        for (i, &joint_index) in group.active_joint_indices().iter().enumerate() {
            let joint = model.joint_model_at(joint_index);
            let first = self.first_variable_index[joint_index];
            let count = joint.variable_count();
            sample_random_positions_near_by(
                joint,
                rng,
                &seed.positions[first..first + count],
                distances[i],
                &mut self.positions[first..first + count],
            );
        }
        self.update_mimic_joints_group(group);
        Ok(())
    }

    // ---- Interpolation --------------------------------------------------

    /// `RobotState::interpolate(to, t, state)`: every **active** joint,
    /// followed by mimic propagation over the whole model.
    ///
    /// Upstream splits this across two files —
    /// `RobotState::interpolate` (`robot_state.cpp:1138`) forwards to
    /// `RobotModel::interpolate` (`robot_model.cpp:1518`), which is the loop
    /// plus `RobotModel::updateMimicJoints`. It is one function here because
    /// the per-joint variable offsets that loop needs
    /// (`active_joint_model_start_index_`) are this type's bookkeeping in
    /// this port, not [`RobotModel`]'s; splitting it would mean publishing
    /// that offset table from [`RobotModel`] to serve one caller.
    ///
    /// The mimic step is why this is not the same as calling
    /// [`JointModel::interpolate`](cspace_model::joint::JointModel::interpolate)
    /// per joint: a mimic joint's interpolated value is **not** its own
    /// interpolation between `from` and `to`, it is
    /// `factor * interpolated_master + offset`. The two agree only when the
    /// master's interpolation is affine in `t`, which a continuous revolute
    /// taking the wrap branch and a floating joint's slerp are not.
    ///
    /// # Errors
    ///
    /// [`Error::Other`] if `t` is NaN or infinite, matching upstream's
    /// `checkInterpolationParamBounds` throwing `moveit::Exception`. A `t`
    /// outside `[0, 1]` is *not* an error there — it logs and extrapolates —
    /// so it is not one here either.
    pub fn interpolate(&self, to: &Self, t: f64, state: &mut Self) -> Result<()> {
        check_interpolation_param_bounds(t)?;
        for &joint_index in self.model.active_joint_indices() {
            self.interpolate_one(to, t, state, joint_index);
        }
        state.propagate_all_mimics();
        state.dirty = Some(state.root_joint_index);
        Ok(())
    }

    /// `RobotState::interpolate(to, t, state, group)`
    /// (`robot_state.cpp:1147`): the group's active joints, then
    /// `RobotState::updateMimicJoints(group)` — which walks
    /// `group->getMimicJointModels()`, **the group's** mimic joints, not the
    /// model's. A mimic whose group does not contain it keeps whatever value
    /// `state` already held, which is the one place the whole-model form
    /// above and this one disagree on the same inputs.
    ///
    /// # Errors
    ///
    /// [`Error::Other`] if `t` is NaN or infinite;
    /// [`Error::UnknownName`] if no group is named `group`.
    pub fn interpolate_group(
        &self,
        to: &Self,
        t: f64,
        state: &mut Self,
        group: &str,
    ) -> Result<()> {
        check_interpolation_param_bounds(t)?;
        let group = self.model.joint_model_group(group)?;
        for &joint_index in group.active_joint_indices() {
            self.interpolate_one(to, t, state, joint_index);
            state.mark_dirty(joint_index);
        }
        for &mimic_index in group.mimic_joint_indices() {
            state.write_mimic(mimic_index);
        }
        Ok(())
    }

    /// `RobotState::interpolate(to, t, state, joint)`
    /// (`robot_state.cpp:1159`): one joint, then the joints that mimic it.
    ///
    /// This overload alone does **not** call
    /// `checkInterpolationParamBounds`: the other two open with it, this one
    /// opens with the zero-variable early return and then goes straight to
    /// `joint->interpolate`. So a NaN `t` throws through the whole-model and
    /// group forms and propagates as NaN positions through this one, and the
    /// asymmetry is upstream's, not a port simplification.
    ///
    /// # Errors
    ///
    /// [`Error::UnknownName`] if no joint is named `joint`.
    pub fn interpolate_joint(
        &self,
        to: &Self,
        t: f64,
        state: &mut Self,
        joint: &str,
    ) -> Result<()> {
        let joint_index = self.joint_index(joint)?;
        if self.model.joint_model_at(joint_index).variable_count() == 0 {
            return Ok(());
        }
        self.interpolate_one(to, t, state, joint_index);
        state.mark_dirty(joint_index);
        state.update_mimic_joint(joint_index);
        Ok(())
    }

    /// One joint's variables, `from` and `to` read out of `self` and the
    /// argument, written into `state`. `state` may alias neither, so the
    /// slices are staged through a fixed-capacity buffer rather than
    /// borrowed — upstream indexes three distinct `position_` arrays and has
    /// no such constraint.
    fn interpolate_one(&self, to: &Self, t: f64, state: &mut Self, joint_index: JointIndex) {
        let joint = self.model.joint_model_at(joint_index);
        let first = self.first_variable_index[joint_index];
        let count = joint.variable_count();
        let mut out = [0.0; MAX_JOINT_VARIABLES];
        joint.interpolate(
            &self.positions[first..first + count],
            &to.positions[first..first + count],
            t,
            &mut out[..count],
        );
        state.positions[first..first + count].copy_from_slice(&out[..count]);
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
            if self.mimic_master_index[joint_index].is_some() {
                self.write_mimic(joint_index);
            }
        }
    }

    /// `values[dest] = values[src] * factor + offset`, and the dirty mark
    /// that goes with it — the one place this port writes a mimic variable.
    /// Upstream writes the same expression in three places
    /// (`RobotModel::updateMimicJoints`, `RobotState::updateMimicJoint`,
    /// `RobotState::updateMimicJoints`), and the port had two of them; a
    /// third for [`RobotState::interpolate_group`] would have made a wrong
    /// factor/offset order a thing you can fix in one caller and still ship
    /// in the others.
    ///
    /// Marking is done here rather than by the caller for the same reason:
    /// two of upstream's three sites mark the follower dirty and the third
    /// (`RobotModel`'s, which works on a bare `double*`) has no dirty state
    /// to mark, so a caller-marks design has one site whose omission is
    /// correct and two whose omission is a stale-transform bug.
    ///
    /// # Panics
    ///
    /// If `mimic_index` does not name a mimic joint. Every caller reaches
    /// this through `mimic_master_index`, `mimic_requests` or a group's
    /// `mimic_joint_indices`, all three of which list only mimic joints.
    fn write_mimic(&mut self, mimic_index: JointIndex) {
        let mimic = self
            .model
            .joint_model_at(mimic_index)
            .mimic()
            .expect("mimic_index names a mimic joint");
        let master_index = self.mimic_master_index[mimic_index]
            .expect("a joint with a mimic has a resolved master");
        let source = self.positions[self.first_variable_index[master_index]];
        self.positions[self.first_variable_index[mimic_index]] =
            mimic.factor * source + mimic.offset;
        self.mark_dirty(mimic_index);
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
        for follower_index in self.mimic_requests[joint_index].clone() {
            self.write_mimic(follower_index);
        }
    }

    /// `RobotState::updateMimicJoints(const JointModelGroup*)`: every mimic
    /// joint in `group`, derived from its master's *current* value, plus a
    /// dirty mark on every one of `group`'s own active joints. Used by
    /// [`RobotState::set_to_random_positions_near_by_group`] and
    /// [`RobotState::set_to_random_positions_near_by_group_with_distances`]
    /// after they have already written every active joint's value.
    ///
    /// Upstream's version marks each mimic joint dirty individually (which
    /// [`RobotState::write_mimic`] already does here) and then marks the
    /// group's *own* active joints dirty too, expanding the tracked dirty
    /// region to their common root
    /// (`markDirtyJointTransforms(const JointModelGroup*)`,
    /// `robot_state.hpp:1686`). This port has no per-joint dirty-transform
    /// array to set — [`RobotState::dirty`] already collapses to a single
    /// common-root marker — so folding every active joint through
    /// [`RobotState::mark_dirty`] is the literal translation of that
    /// expansion, not an approximation of it.
    fn update_mimic_joints_group(&mut self, group: &JointModelGroup) {
        for &mimic_index in group.mimic_joint_indices() {
            self.write_mimic(mimic_index);
        }
        for &joint_index in group.active_joint_indices() {
            self.mark_dirty(joint_index);
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
    // `is_chain`/`joint_roots` (`JointModelGroup::is_chain`/`joint_roots`)
    // and the descendant-link walk (`RobotModel::descendant_link_indices`)
    // are now computed once, at model-build time, by `cspace-model` itself
    // (see those methods' own doc comments) -- this crate used to duplicate
    // both traversals here on demand, before `cspace-model` carried them.
    // `jacobian`, this section's only caller, uses them directly instead.

    /// The link a joint is attached from. Upstream
    /// `JointModel::getParentLinkModel()`; `None` only for the model's
    /// absolute root joint, matching upstream returning `nullptr` there.
    fn parent_link_of_joint(&self, joint_index: JointIndex) -> Option<usize> {
        self.model
            .link_model_at(self.link_of_joint[joint_index])
            .parent_link_index()
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
    /// link in [`RobotModel::link_models`](cspace_model::RobotModel::link_models)
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

    /// `getFrameTransform`/`getFrameInfo`, restricted to what this crate
    /// alone can resolve: a leading `/` is stripped, `frame_id == model_frame`
    /// resolves to the identity transform at the root link (upstream:
    /// `robot_state.cpp:1345`), and otherwise `frame_id` must name a link.
    /// Upstream's further fallback to attached bodies and their subframes
    /// lives one layer up, on the `cspace-scene` crate's
    /// `PlanningScene::frame_transform` — this port keeps attached bodies on
    /// the scene rather than on `RobotState` (see that crate's
    /// `attached_body` module doc), so this method structurally cannot see
    /// them; the scene calls this method for its own first two tiers before
    /// trying the rest.
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
    ///    already implies at least one active joint (see
    ///    [`cspace_model::JointModelGroup::is_chain`]'s own doc comment), so
    ///    it is not ported as a separate case.
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

        if !group_model.is_chain() {
            return Err(Error::other(format!(
                "the group '{group}' is not a chain; cannot compute Jacobian"
            )));
        }
        let chain_root = *group_model
            .joint_roots()
            .first()
            .expect("is_chain() implies exactly one joint root");

        let tip_link = *group_model
            .link_indices()
            .last()
            .expect("a chain root's active joint gives the group at least one link");

        let descendant_links = model.descendant_link_indices(chain_root);
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
/// port's [`cspace_model::RobotModel`] deliberately excludes it (Phase 1
/// deviation #4: the C++ oracle owns randomness for differential testing).
/// Mirrors each joint kind's actual upstream sampling rule (verified by
/// reading `{revolute,prismatic,planar,floating}_joint_model.cpp`, not
/// assumed from the joint's general shape):
///
/// - Revolute/prismatic: uniform within bounds.
/// - Planar translation (x, y) and floating translation (x, y, z): uniform
///   within bounds, or `0.0` if that axis's bounds are non-finite (a
///   floating joint's translation is `position_bounded == true` with
///   infinite `min`/`max` — see [`cspace_model::joint::FloatingJoint`]'s
///   doc comment — so "bounded" cannot be used as the non-finite check
///   here; finiteness is checked directly).
/// - Planar rotation (theta): uniform within bounds directly, no
///   finiteness check — a planar joint's theta bounds are always finite
///   (`[-pi, pi]`) even though `position_bounded == false` marks it
///   unbounded (it wraps, per [`crate`]'s use of
///   [`cspace_model::joint::PlanarJoint`]).
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

/// `JointModel::getVariableRandomPositionsNearBy`, dispatched per joint
/// kind (upstream splits this across `RevoluteJointModel`,
/// `PrismaticJointModel`, `PlanarJointModel`, `FloatingJointModel`,
/// `FixedJointModel`, each overriding the base class's pure-virtual
/// method). Mirrors [`sample_random_positions`]'s per-kind dispatch shape;
/// unlike that function, every non-fixed kind's sample here is drawn from
/// `[near - distance, near + distance]` (clamped to bounds, or wrapped for
/// a continuous revolute), not from the joint's full range.
///
/// - Revolute, continuous: unclamped uniform in `[near - distance, near +
///   distance]`, then wrapped into `(-pi, pi]` via
///   [`JointModel::enforce_position_bounds`] on the joint's own bounds —
///   matching `crates/cspace-kinematics/src/cart_to_jnt.rs`'s
///   `near_by_configuration`, which ports this same case for the KDL path.
/// - Revolute (non-continuous) and prismatic: uniform in `[near -
///   distance, near + distance]`, clamped to bounds.
/// - Planar/floating translation: as above, or `0.0` if that axis's
///   bounds are non-finite (see [`sample_random_positions`]'s doc comment
///   for why "bounded" cannot be used as the finiteness check here).
/// - Planar rotation: uniform in `[near - da, near + da]` where `da =
///   angular_distance_weight * distance` clamped to `pi`, then wrapped
///   into `[-pi, pi]` via [`PlanarJoint::normalize_rotation`] — *not*
///   clamped to bounds, matching upstream (which calls `normalizeRotation`,
///   not `enforcePositionBounds`, here).
/// - Floating rotation: a small rotation composed onto `near`'s
///   quaternion. When `da = angular_distance_weight * distance >= pi/4`,
///   upstream gives up on "near" and draws a fully random unit quaternion
///   instead (matched here via [`sample_unit_quaternion`]). Otherwise it
///   draws a random axis and an angle via OMPL's rejection-free
///   ball-sampling (see [`sample_near_axis_angle_quaternion`]) and
///   left-multiplies `near`'s quaternion by the resulting small rotation,
///   using upstream's exact component-wise Hamilton product rather than
///   going through a quaternion type — `values[3..7]` is `(x, y, z, w)`,
///   matching [`sample_unit_quaternion`]'s own component order, and `near`
///   is assumed to already be in that order (as every caller's `position_`
///   slice for a floating joint is).
///
/// # Panics
///
/// Upstream reads `distance` (and, for planar/floating,
/// `angular_distance_weight * distance`) unchecked into `uniformReal(low,
/// high)`, whose `low <= high` precondition a negative `distance` can
/// violate. This port matches that: a negative `distance` can make `near -
/// distance > near + distance`, and [`RngExt::random_range`] panics on an
/// empty range rather than silently swapping the bounds.
fn sample_random_positions_near_by(
    joint: &JointModel,
    rng: &mut impl Rng,
    near: &[f64],
    distance: f64,
    out: &mut [f64],
) {
    let bounds = joint.variable_bounds();
    match joint.kind() {
        JointKind::Revolute(r) => {
            if r.is_continuous() {
                out[0] = rng.random_range((near[0] - distance)..=(near[0] + distance));
                joint.enforce_position_bounds(out);
            } else {
                out[0] = sample_uniform_near_by(
                    rng,
                    bounds[0].min_position,
                    bounds[0].max_position,
                    near[0],
                    distance,
                );
            }
        }
        JointKind::Prismatic(_) => {
            out[0] = sample_uniform_near_by(
                rng,
                bounds[0].min_position,
                bounds[0].max_position,
                near[0],
                distance,
            );
        }
        JointKind::Planar(p) => {
            out[0] = sample_uniform_or_zero_near_by(
                rng,
                bounds[0].min_position,
                bounds[0].max_position,
                near[0],
                distance,
            );
            out[1] = sample_uniform_or_zero_near_by(
                rng,
                bounds[1].min_position,
                bounds[1].max_position,
                near[1],
                distance,
            );
            // `cxx_min`, not `f64::min`: upstream's `if (da > M_PI) da = M_PI;`
            // keeps a NaN `da` as NaN (the comparison is false, so the
            // assignment never runs) — see `crate::numeric`.
            let da = cxx_min(p.angular_distance_weight() * distance, PI);
            out[2] = rng.random_range((near[2] - da)..=(near[2] + da));
            let out3: &mut [f64; 3] = (&mut out[..3])
                .try_into()
                .expect("planar joint has 3 variables");
            PlanarJoint::normalize_rotation(out3);
        }
        JointKind::Floating(f) => {
            out[0] = sample_uniform_or_zero_near_by(
                rng,
                bounds[0].min_position,
                bounds[0].max_position,
                near[0],
                distance,
            );
            out[1] = sample_uniform_or_zero_near_by(
                rng,
                bounds[1].min_position,
                bounds[1].max_position,
                near[1],
                distance,
            );
            out[2] = sample_uniform_or_zero_near_by(
                rng,
                bounds[2].min_position,
                bounds[2].max_position,
                near[2],
                distance,
            );
            let da = f.angular_distance_weight() * distance;
            if da >= 0.25 * PI {
                let (x, y, z, w) = sample_unit_quaternion(rng);
                out[3] = x;
                out[4] = y;
                out[5] = z;
                out[6] = w;
            } else {
                let (qx, qy, qz, qw) = sample_near_axis_angle_quaternion(rng, da);
                // Hamilton product `near * q`, upstream's exact formula
                // (`floating_joint_model.cpp`): near = (near[3..7]) =
                // (x, y, z, w), q = (qx, qy, qz, qw).
                out[3] = near[6] * qx + near[3] * qw + near[4] * qz - near[5] * qy;
                out[4] = near[6] * qy + near[4] * qw + near[5] * qx - near[3] * qz;
                out[5] = near[6] * qz + near[5] * qw + near[3] * qy - near[4] * qx;
                out[6] = near[6] * qw - near[3] * qx - near[4] * qy - near[5] * qz;
            }
        }
        JointKind::Fixed => {}
    }
}

/// Uniform in `[near - distance, near + distance]`, clamped to `[min,
/// max]`. `FloatingJointModel`/`PlanarJointModel`/`PrismaticJointModel`/
/// `RevoluteJointModel::getVariableRandomPositionsNearBy`'s common
/// `uniformReal(std::max(min, near - distance), std::min(max, near +
/// distance))` shape.
///
/// `cxx_max`/`cxx_min`, not `f64::max`/`f64::min`: upstream keeps a NaN
/// `min`/`max` bound as NaN (`std::max`/`std::min`'s first argument), while
/// `f64::max`/`f64::min` would silently discard it in favor of `near ±
/// distance`. Reachable for the Revolute (non-continuous) and Prismatic
/// callers, which pass a joint's `min_position`/`max_position` here
/// directly with no `is_finite` screen — unlike the Planar/Floating
/// callers, which route through `sample_uniform_or_zero_near_by`'s
/// `is_finite` check first.
fn sample_uniform_near_by(rng: &mut impl Rng, min: f64, max: f64, near: f64, distance: f64) -> f64 {
    sample_uniform(
        rng,
        cxx_max(min, near - distance),
        cxx_min(max, near + distance),
    )
}

/// As [`sample_uniform_near_by`], or `0.0` if `min`/`max` are not both
/// finite — see [`sample_random_positions_near_by`]'s doc comment.
fn sample_uniform_or_zero_near_by(
    rng: &mut impl Rng,
    min: f64,
    max: f64,
    near: f64,
    distance: f64,
) -> f64 {
    if min.is_finite() && max.is_finite() {
        sample_uniform_near_by(rng, min, max, near, distance)
    } else {
        0.0
    }
}

/// A small rotation as `(x, y, z, w)`, for `FloatingJointModel`'s
/// near-by sampling when `da < pi/4`. Upstream (comment: "taken from
/// OMPL"): draw a random axis from 3 iid standard-normal components
/// (normalized; the identity quaternion if the draw norm is under
/// `1e-6`), and an angle `2 * cbrt(u) * da` for `u` uniform in `[0, 1)` —
/// rejection-free sampling of a ball of radius `da` in the tangent space,
/// then wrapped onto `SO(3)` via the half-angle. This port uses
/// [`f64::cbrt`] where upstream writes `pow(u, 1.0/3.0)`; the two agree
/// for `u >= 0` (guaranteed here) and `cbrt` is the more direct
/// translation of "cube root", not a behavior change.
fn sample_near_axis_angle_quaternion(rng: &mut impl Rng, da: f64) -> (f64, f64, f64, f64) {
    let ax: f64 = rng.sample(StandardNormal);
    let ay: f64 = rng.sample(StandardNormal);
    let az: f64 = rng.sample(StandardNormal);
    let angle = 2.0 * rng.random::<f64>().cbrt() * da;
    let norm = (ax * ax + ay * ay + az * az).sqrt();
    if norm < 1e-6 {
        (0.0, 0.0, 0.0, 1.0)
    } else {
        let s = (angle / 2.0).sin();
        (
            s * ax / norm,
            s * ay / norm,
            s * az / norm,
            (angle / 2.0).cos(),
        )
    }
}

// This crate's other tests all live in `tests/*.rs`, exercising the public
// `RobotState` API through URDF/SRDF fixtures (see e.g.
// `tests/group_joint_values.rs`). `sample_random_positions_near_by`'s
// per-joint-kind branches are boundary-tested directly here instead,
// matching how `cspace_model::joint::{RevoluteJoint, PlanarJoint,
// FloatingJoint}` test their own analogous per-kind logic: no fixture in
// this crate's `tests/fixtures/` puts a floating or a continuous-revolute
// joint inside an SRDF group, so a fixture-only test could not reach two
// of the five kinds at all, and even where a fixture exists, an RNG-driven
// integration test could only check post-hoc properties (bounds
// satisfied), not pin the exact branch (continuous wrap vs. clamp,
// infinite-bounds-zero, small-vs-large `da`) a given input takes.
#[cfg(test)]
mod near_by_sampling_tests {
    use approx::assert_relative_eq;
    use rand::SeedableRng;
    use rand_chacha::ChaCha8Rng;

    use cspace_model::joint::VariableBounds;

    use super::*;

    #[test]
    fn revolute_continuous_near_by_wraps_past_pi_instead_of_clamping() {
        let mut joint = JointModel::new_revolute("j");
        joint.set_continuous(true).unwrap();
        let mut rng = ChaCha8Rng::seed_from_u64(0);
        let mut out = [0.0];
        // distance = 0.0 collapses the near-by window to the single point
        // `near`, so the sampled value before wrapping is deterministic
        // regardless of the RNG draw.
        sample_random_positions_near_by(&joint, &mut rng, &[PI + 0.5], 0.0, &mut out);
        assert_relative_eq!(out[0], -PI + 0.5, epsilon = 1e-12);
    }

    #[test]
    #[should_panic]
    fn near_by_panics_on_a_negative_distance_for_a_continuous_revolute() {
        let mut joint = JointModel::new_revolute("j");
        joint.set_continuous(true).unwrap();
        let mut rng = ChaCha8Rng::seed_from_u64(0);
        let mut out = [0.0];
        // A negative distance makes `near - distance > near + distance`,
        // an empty range -- upstream reads this unchecked into
        // `uniformReal`'s `low <= high` precondition (see this function's
        // `# Panics` doc comment); this port's `random_range` panics
        // instead of silently swapping the bounds.
        sample_random_positions_near_by(&joint, &mut rng, &[0.0], -1.0, &mut out);
    }

    #[test]
    fn revolute_non_continuous_near_by_clamps_to_the_joint_bounds() {
        let joint = JointModel::new_revolute("j"); // bounds [-pi, pi], not continuous
        let mut rng = ChaCha8Rng::seed_from_u64(0);
        let mut out = [0.0];
        // near - distance = pi (inside), near + distance = pi + 2.0
        // (outside) -- the clamp collapses the window to the single point
        // `pi`, the upper bound itself.
        sample_random_positions_near_by(&joint, &mut rng, &[PI + 1.0], 1.0, &mut out);
        assert_eq!(out[0], PI);
    }

    #[test]
    fn prismatic_near_by_clamps_to_bounds() {
        let mut joint = JointModel::new_prismatic("j");
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
        let mut rng = ChaCha8Rng::seed_from_u64(0);
        let mut out = [0.0];
        // near - distance = 1.0 (== the upper bound), near + distance =
        // 3.0 (outside) -- the clamp collapses the window to the single
        // point `1.0`.
        sample_random_positions_near_by(&joint, &mut rng, &[2.0], 1.0, &mut out);
        assert_eq!(out[0], 1.0);
    }

    #[test]
    #[should_panic]
    fn near_by_panics_on_a_negative_distance_that_empties_the_range() {
        let joint = JointModel::new_prismatic("j");
        let mut rng = ChaCha8Rng::seed_from_u64(0);
        let mut out = [0.0];
        sample_random_positions_near_by(&joint, &mut rng, &[0.0], -1.0, &mut out);
    }

    #[test]
    fn planar_translation_near_by_is_zero_when_bounds_are_infinite() {
        let joint = JointModel::new_planar("j"); // x/y bounds default to +/-inf
        let mut rng = ChaCha8Rng::seed_from_u64(0);
        let mut out = [0.0; 3];
        sample_random_positions_near_by(&joint, &mut rng, &[5.0, -3.0, 0.0], 1.0, &mut out);
        assert_eq!(out[0], 0.0);
        assert_eq!(out[1], 0.0);
    }

    #[test]
    fn planar_rotation_near_by_wraps_via_normalize_rotation() {
        let joint = JointModel::new_planar("j");
        let mut rng = ChaCha8Rng::seed_from_u64(0);
        let mut out = [0.0; 3];
        // distance = 0.0 collapses `da` to 0.0 too, so `out[2]` is `near[2]`
        // itself before `normalize_rotation` wraps it back into [-pi, pi].
        sample_random_positions_near_by(&joint, &mut rng, &[0.0, 0.0, PI + 0.1], 0.0, &mut out);
        assert_relative_eq!(out[2], -PI + 0.1, epsilon = 1e-12);
    }

    /// A NaN `angular_distance_weight` (the `cxx_min` receiver, matching
    /// upstream's `if (da > M_PI) da = M_PI;` leaving a NaN `da` untouched)
    /// must keep `da` as NaN, not be silently clamped to `PI`. Tested on
    /// the exact production expression rather than through
    /// `sample_random_positions_near_by`'s full draw: a NaN `da` there
    /// widens `rng.random_range` to a NaN-bounded range, which panics
    /// downstream regardless of which side of this fix produced it, so
    /// `da` itself — not a value two calls further down — is what this
    /// fix is about. `f64::min` discards the NaN; this fails before the
    /// `cxx_min` fix and passes after.
    #[test]
    fn planar_rotation_clamp_propagates_nan_from_angular_distance_weight() {
        let mut joint = JointModel::new_planar("j");
        joint
            .as_planar_mut()
            .unwrap()
            .set_angular_distance_weight(f64::NAN);
        let JointKind::Planar(p) = *joint.kind() else {
            panic!("just constructed as planar")
        };
        let da = cxx_min(p.angular_distance_weight() * 1.0, PI);
        assert!(da.is_nan());
    }

    /// Demonstrated opposite: a finite `angular_distance_weight` that
    /// overshoots `PI` is still clamped to `PI`, on both sides of the fix.
    /// Without this, a fix that made `da` unconditionally NaN would also
    /// pass the test above.
    #[test]
    fn planar_rotation_clamp_still_clamps_a_finite_overshoot_to_pi() {
        let mut joint = JointModel::new_planar("j");
        joint
            .as_planar_mut()
            .unwrap()
            .set_angular_distance_weight(10.0);
        let JointKind::Planar(p) = *joint.kind() else {
            panic!("just constructed as planar")
        };
        let da = cxx_min(p.angular_distance_weight() * 1.0, PI);
        assert_eq!(da, PI);
    }

    /// A NaN `min`/`max` bound (the `cxx_max`/`cxx_min` receiver, matching
    /// upstream's `std::max`/`std::min` first argument) must survive into
    /// `sample_uniform`'s range, not be silently discarded in favor of
    /// `near ± distance`. Observed as a panic from `rng.random_range` on
    /// the resulting NaN-bounded range (an empty range by `PartialOrd`),
    /// not as a returned value: whatever upstream's own
    /// `uniform_real_distribution<double>(NaN, finite)` does with a NaN
    /// parameter is undefined behavior and out of scope for this fix,
    /// which is about whether the NaN reaches the range constructor at
    /// all. `f64::max` would have discarded it, producing a plausible
    /// finite draw with no panic; this fails (does not panic) before the
    /// `cxx_max` fix and panics after.
    #[test]
    #[should_panic(expected = "cannot sample empty range")]
    fn sample_uniform_near_by_propagates_nan_from_the_min_bound() {
        let mut rng = ChaCha8Rng::seed_from_u64(0);
        sample_uniform_near_by(&mut rng, f64::NAN, 10.0, 5.0, 1.0);
    }

    /// As above, for the `max` bound reaching `cxx_min`.
    #[test]
    #[should_panic(expected = "cannot sample empty range")]
    fn sample_uniform_near_by_propagates_nan_from_the_max_bound() {
        let mut rng = ChaCha8Rng::seed_from_u64(0);
        sample_uniform_near_by(&mut rng, 0.0, f64::NAN, 5.0, 1.0);
    }

    /// Demonstrated opposite: a NaN `distance` (poisoning `near ±
    /// distance`, the `cxx_max`/`cxx_min` non-receiver) is discarded in
    /// favor of the finite `min`/`max` bounds on both sides of the fix, so
    /// a normal in-bounds draw still comes out — no panic. Without this, a
    /// fix that made the clamp unconditionally NaN-propagating regardless
    /// of which operand carried the NaN would also "pass" the two tests
    /// above (by panicking on everything).
    #[test]
    fn sample_uniform_near_by_discards_nan_from_distance() {
        let mut rng = ChaCha8Rng::seed_from_u64(0);
        let result = sample_uniform_near_by(&mut rng, 0.0, 10.0, 5.0, f64::NAN);
        assert!((0.0..=10.0).contains(&result));
    }

    #[test]
    fn floating_translation_near_by_is_zero_when_bounds_are_infinite() {
        let joint = JointModel::new_floating("j"); // translation bounds default to +/-inf
        let mut rng = ChaCha8Rng::seed_from_u64(0);
        let mut out = [0.0; 7];
        let near = [5.0, -3.0, 2.0, 0.0, 0.0, 0.0, 1.0];
        sample_random_positions_near_by(&joint, &mut rng, &near, 1.0, &mut out);
        assert_eq!(out[0], 0.0);
        assert_eq!(out[1], 0.0);
        assert_eq!(out[2], 0.0);
    }

    #[test]
    fn floating_rotation_near_by_produces_a_unit_quaternion_when_da_is_large() {
        let mut joint = JointModel::new_floating("j");
        joint
            .as_floating_mut()
            .unwrap()
            .set_angular_distance_weight(1.0);
        let mut rng = ChaCha8Rng::seed_from_u64(0);
        let mut out = [0.0; 7];
        let near = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0];
        // da = angular_distance_weight * distance = 1.0 >= pi/4, so upstream
        // gives up on "near" and draws a fully random unit quaternion.
        sample_random_positions_near_by(&joint, &mut rng, &near, 1.0, &mut out);
        let norm_squared = out[3] * out[3] + out[4] * out[4] + out[5] * out[5] + out[6] * out[6];
        assert_relative_eq!(norm_squared, 1.0, epsilon = 1e-9);
    }

    #[test]
    fn floating_rotation_near_by_is_the_identity_perturbation_when_distance_is_zero() {
        let joint = JointModel::new_floating("j");
        let mut rng = ChaCha8Rng::seed_from_u64(0);
        let mut out = [0.0; 7];
        // A non-identity, already-normalized "near" quaternion.
        let near = [0.0, 0.0, 0.0, 0.6, 0.0, 0.0, 0.8];
        // distance = 0.0 forces da = 0.0, so the composed small rotation's
        // half-angle is exactly 0 regardless of the sampled axis -- the
        // small-rotation branch's `sin(angle/2)` term zeroes out `qx, qy,
        // qz` and `near`'s quaternion passes through the Hamilton product
        // unchanged (`near * identity == near`), landing on the exact
        // input rather than merely something close to it.
        sample_random_positions_near_by(&joint, &mut rng, &near, 0.0, &mut out);
        assert_eq!(out[3], near[3]);
        assert_eq!(out[4], near[4]);
        assert_eq!(out[5], near[5]);
        assert_eq!(out[6], near[6]);
    }

    #[test]
    fn fixed_near_by_is_a_no_op() {
        let joint = JointModel::new_fixed("j");
        let mut rng = ChaCha8Rng::seed_from_u64(0);
        let mut out: [f64; 0] = [];
        // Must not panic on the empty `near`/`out` slices.
        sample_random_positions_near_by(&joint, &mut rng, &[], 1.0, &mut out);
    }
}
