// Copyright (c) 2011, Willow Garage, Inc.
// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// Ported from moveit2 @ e017c91ee12984393a28ba246075c65f69cde3bf:
//   moveit_core/constraint_samplers/include/moveit/constraint_samplers/constraint_sampler.hpp
//   moveit_core/constraint_samplers/include/moveit/constraint_samplers/default_constraint_samplers.hpp
//   moveit_core/constraint_samplers/src/default_constraint_samplers.cpp
//   (class JointConstraintSampler, JointConstraintSampler::configure,
//   JointConstraintSampler::sample)
//   moveit_core/constraint_samplers/include/moveit/constraint_samplers/union_constraint_sampler.hpp
//   moveit_core/constraint_samplers/src/union_constraint_sampler.cpp
//   (class UnionConstraintSampler, struct OrderSamplers)

//! [`ConstraintSampler`] and the two samplers `PORTING-PLAN.md`'s
//! `registry.rs` disposition (`crates/cspace-planners/src/sbp/registry.rs`)
//! identified as needing no new dependency: [`JointConstraintSampler`] and
//! [`UnionConstraintSampler`]. `IKConstraintSampler` is ported in
//! `crate::constraints::ik_sampler` as [`crate::constraints::IkConstraintSampler`], not here — it needs
//! `cspace_core::kinematics::KinematicsSolver`, a real new dependency edge this
//! module's two samplers still do not, and — per this module's own doc
//! comment below on [`ConstraintSampler::sample`]'s collapsed signature —
//! it does not implement this trait at all.
//!
//! # No `PlanningScene`
//!
//! Upstream's `ConstraintSampler` base class is constructed from a
//! `planning_scene::PlanningSceneConstPtr`, using it only to reach
//! `scene_->getRobotModel()` (to resolve `group_name` into a `JointModelGroup`)
//! and, for `IKConstraintSampler`, `scene_->getTransforms()`. Neither
//! [`JointConstraintSampler`] nor [`UnionConstraintSampler`] needs anything
//! else a `PlanningScene` provides (no collision checking, no attached
//! bodies), so both take `&RobotModel` at construction instead — the same
//! substitution [`crate::constraints::JointConstraint::new`] and friends already make,
//! and it means this crate still needs no `cspace_planning::scene` dependency.
//!
//! # `constraint_sampler.cpp`: where its two function bodies went
//!
//! Upstream's base-class *implementation* file is 67 lines and holds
//! exactly two bodies. Both are accounted for here, neither as a
//! transcription:
//!
//! - `ConstraintSampler::ConstraintSampler(scene, group_name)`
//!   (`constraint_sampler.cpp:52-60`) does one substantive thing:
//!   `jmg_ = scene->getRobotModel()->getJointModelGroup(group_name)`,
//!   followed by an `RCLCPP_ERROR` when that lookup returns null — and then
//!   construction proceeds anyway with a null `jmg_`, which every
//!   `configure()` re-checks (`default_constraint_samplers.cpp:72`). That
//!   lookup is the first line of [`JointConstraintSampler::new`] and
//!   [`UnionConstraintSampler::new`], `model.joint_model_group(group_name)?`,
//!   where the `?` turns log-and-continue-with-a-null into
//!   [`Error::UnknownName`] at construction
//!   ([`crate::constraints::IkConstraintSamplerAdapter::new`] is handed an
//!   already-resolved `&JointModelGroup` and has no lookup to do). The
//!   remaining three initialisers — `is_valid_(false)`, `verbose_(false)`,
//!   `scene_(scene)` — initialise fields this port has no equivalent of;
//!   each is dispositioned by name in this crate's
//!   `constraint_samplers/*.hpp` symbol audit (see `crate`'s doc comment).
//! - `ConstraintSampler::clear()` (`:62-66`) resets `is_valid_` and
//!   `frame_depends_`. It is reached from exactly two places, both inside
//!   `configure()`: the top of `JointConstraintSampler::configure(jc)` and
//!   of `IKConstraintSampler::configure(sp)`
//!   (`default_constraint_samplers.cpp:70,255`, via each type's own
//!   `clear()` override), so a *second* `configure()` on a live sampler
//!   starts blank; and `:121`, the "no possible values for the joint"
//!   failure path, which must hand-undo the partial configuration it had
//!   already written into a sampler that outlives the failure. This port
//!   has neither caller by construction: a sampler is built whole by a
//!   fallible `new()` with no reconfigure step, `frame_depends` is computed
//!   once inside that `new()` and never written again, and the `:121`
//!   failure is [`JointConstraintSampler::new`]'s `min_bound > max_bound`
//!   `return Err(..)`, after which no partially-built value exists to
//!   reset. Deliberately not ported — `PORTING-PLAN.md` §225.2.
//!
//! # `sample`'s collapsed signature: no separate `reference_state`, no `max_attempts`
//!
//! Upstream's virtual `sample(state, reference_state, max_attempts)` exists
//! so `IKConstraintSampler` can seed IK from a `reference_state` distinct
//! from the `state` being written into, and can retry up to `max_attempts`
//! times. Neither ported sampler here reads `reference_state` as anything
//! but "the same object as `state`" ([`JointConstraintSampler::sample`]
//! ignores it entirely, matching upstream's own `/* reference_state */`
//! comment; [`UnionConstraintSampler::sample`] only ever assigns `state =
//! reference_state` once at the top, upstream's own convenience overload for
//! the common case where they *are* the same object), and neither reads
//! `max_attempts` either (`JointConstraintSampler` has no retry loop at all
//! — "we are always successful" — and `UnionConstraintSampler` only
//! forwards it unread). Taking a `&mut` and a `&` to the same `RobotState`
//! simultaneously does not typecheck in Rust in the first place, so a
//! literal port of the 3-arg signature would force every real caller of the
//! common case through a clone. [`ConstraintSampler::sample`] therefore
//! takes one `&mut RobotState`, no separate reference and no attempt count.
//! This is a deliberate, narrower trait than upstream's base class.
//! [`crate::constraints::IkConstraintSampler`] is the case that actually needs both a
//! distinct reference state and a retry budget, confirming the prediction
//! this paragraph used to make — but it does not widen this trait to get
//! them; see its own doc comment for why an inherent method (not a wider
//! [`ConstraintSampler::sample`] every implementer would have to grow unread
//! parameters for) was the better fit once a real second caller existed.

use std::cmp::Ordering;
use std::collections::{BTreeMap, BTreeSet};

use cspace_core::error::{Error, Result};
use cspace_core::model::{JointModelGroup, RobotModel};
use cspace_core::state::RobotState;
use rand::{Rng, RngExt};

/// Upstream `ConstraintSampler::DEFAULT_MAX_SAMPLING_ATTEMPTS`
/// (`constraint_sampler.hpp:64`), `2`. Upstream's only two uses of this
/// constant (`constraint_sampler.hpp:171,202`) are default arguments to two
/// `sample()` overloads this port's own trait design already collapses away
/// (see this module's doc comment) — no live production call site upstream
/// ever actually receives `2`; every real caller (`constrained_sampler.cpp:69-70`,
/// `constrained_goal_sampler.cpp:137`) instead passes
/// `ModelBasedPlanningContext::getMaximumStateSamplingAttempts()`, configured
/// to `4` by `planning_context_manager.cpp:259`. Round 20 mistakenly reused
/// this constant as that value in
/// `cspace_planners::sbp::registry::RrtConnectContext::solve`; round 21
/// corrected it to a locally-defined `DEFAULT_MAX_STATE_SAMPLING_ATTEMPTS = 4`
/// there instead, so this constant remains ported (it matches upstream's
/// named literal) but — as rounds 13/14 originally found — has no real
/// production call site in this workspace.
pub const DEFAULT_MAX_SAMPLING_ATTEMPTS: u32 = 2;

use crate::constraints::JointConstraint;
use crate::constraints::numeric::{cxx_max, cxx_min};

/// Upstream `constraint_samplers::ConstraintSampler`, the abstract base
/// every concrete sampler implements. See this module's doc comment for how
/// [`ConstraintSampler::sample`]'s signature narrows upstream's.
pub trait ConstraintSampler {
    /// `getJointModelGroup`.
    fn joint_model_group(&self) -> &JointModelGroup;

    /// `getGroupName`.
    fn group_name(&self) -> &str {
        self.joint_model_group().name()
    }

    /// `getFrameDependency`: the names of the mobile frames whose pose this
    /// sampler needs when [`ConstraintSampler::sample`] is called. Used by
    /// [`UnionConstraintSampler`]'s ordering to run a sampler that depends on
    /// another sampler's link only after that link has been set.
    fn frame_dependency(&self) -> &[String];

    /// Not an upstream method: upstream instead lets
    /// `dynamic_cast<JointConstraintSampler*>` answer this inside
    /// `OrderSamplers` (`union_constraint_sampler.cpp:114-118`). This crate
    /// has no RTTI to dynamic-cast a trait object with, so `OrderSamplers`'s
    /// "prefer sampling JointConstraints first" tie-break asks this instead;
    /// [`JointConstraintSampler`] is the only implementer that overrides it.
    fn is_joint_constraint_sampler(&self) -> bool {
        false
    }

    /// `sample(state, reference_state, max_attempts)`, collapsed — see this
    /// module's doc comment. Draws one sample from the constraint(s) this
    /// sampler holds and writes it into `state`. Always succeeds for
    /// [`JointConstraintSampler`] (matching upstream); can fail for
    /// [`UnionConstraintSampler`] only if a member sampler fails (which, for
    /// the samplers ported so far, cannot happen either — the failure path
    /// exists for a future `IKConstraintSampler` member).
    fn sample(&self, state: &mut RobotState<'_>, rng: &mut dyn Rng) -> bool;
}

/// One constrained variable's tightened sampling range. Upstream's
/// `JointConstraintSampler::JointInfo`, minus `index_`: that field only
/// existed to address into upstream's flat `values_` array for the single
/// bulk `setJointGroupPositions` call; this port writes each variable by
/// name instead (see [`JointConstraintSampler::sample`]), so no index needs
/// to survive `configure`.
#[derive(Debug)]
struct JointInfo {
    variable_name: String,
    min_bound: f64,
    max_bound: f64,
}

/// Samples a [`JointModelGroup`]'s variables subject to a set of
/// [`JointConstraint`]s: constrained variables draw uniformly from the
/// intersection of the joint's own bounds and the constraint's tolerance
/// window, unconstrained variables draw from the joint's own bounds.
///
/// Upstream `constraint_samplers::JointConstraintSampler`.
#[derive(Debug)]
pub struct JointConstraintSampler {
    group: JointModelGroup,
    bounds: Vec<JointInfo>,
    /// Every variable name of every group joint that is not fully covered
    /// by `bounds` — flattened across joints rather than kept as
    /// upstream's `unbounded_`/`uindex_` joint list, because
    /// [`JointConstraintSampler::sample`] reads each variable's random draw
    /// back out of a scratch [`RobotState`] by name (see that method's doc
    /// comment), which needs no per-joint grouping to do correctly.
    unbounded_variable_names: Vec<String>,
}

impl JointConstraintSampler {
    /// Upstream's `JointConstraintSampler(scene, group_name)` constructor
    /// plus `configure(const std::vector<JointConstraint>&)`, collapsed into
    /// one call — the same two-step-to-one-step collapse every constraint
    /// type in this crate already makes (see this crate's introducing doc
    /// comment).
    ///
    /// # Errors
    ///
    /// [`Error::UnknownName`] if `group_name` does not name a group in
    /// `model`.
    /// [`Error::Other`] if no constraint in `constraints` applies to a joint
    /// in the group (upstream: "No valid joint constraints"), or if two
    /// constraints on the same variable (or a variable's tolerance window
    /// against the joint's own bounds) intersect to an empty range —
    /// upstream's "there are no possible values for the joint" — a
    /// configure-time failure, not a sample-time one: the whole
    /// constructed set is discarded rather than degrading to a narrower
    /// sampler.
    pub fn new(
        model: &RobotModel,
        group_name: &str,
        constraints: &[JointConstraint],
    ) -> Result<Self> {
        let group = model.joint_model_group(group_name)?.clone();

        let mut bound_data: BTreeMap<String, (f64, f64)> = BTreeMap::new();
        let mut some_valid_constraint = false;
        for constraint in constraints {
            let joint_name = constraint
                .joint_variable_name()
                .split('/')
                .next()
                .unwrap_or(constraint.joint_variable_name());
            if !group.has_joint_model(joint_name) {
                continue;
            }
            some_valid_constraint = true;

            let joint_model = model.joint_model(joint_name)?;
            let joint_bounds = joint_model.variable_bounds_for(constraint.joint_variable_name())?;
            let min = cxx_max(
                joint_bounds.min_position,
                constraint.desired_joint_position() - constraint.joint_tolerance_below(),
            );
            let max = cxx_min(
                joint_bounds.max_position,
                constraint.desired_joint_position() + constraint.joint_tolerance_above(),
            );

            let entry = bound_data
                .entry(constraint.joint_variable_name().to_string())
                .or_insert((f64::MIN, f64::MAX));
            // `potentiallyAdjustMinMaxBounds` puts the *incoming* bound
            // first — `std::max(min, min_bound_)` — so a NaN `min` survives
            // into the running bound. The accumulator-first spelling this
            // replaces put the NaN second, where both C++ and Rust discard
            // it.
            entry.0 = cxx_max(min, entry.0);
            entry.1 = cxx_min(max, entry.1);
            if entry.0 > entry.1 + f64::EPSILON {
                return Err(Error::other(format!(
                    "JointConstraintSampler: no possible values for joint variable '{}': \
                     min_bound {} > max_bound {}",
                    constraint.joint_variable_name(),
                    entry.0,
                    entry.1
                )));
            }
        }

        if !some_valid_constraint {
            return Err(Error::other(format!(
                "JointConstraintSampler: no joint constraints apply to group '{group_name}'"
            )));
        }

        let bounds: Vec<JointInfo> = bound_data
            .into_iter()
            .map(|(variable_name, (min_bound, max_bound))| JointInfo {
                variable_name,
                min_bound,
                max_bound,
            })
            .collect();

        // `jmg_->getJointModels()` filtered to `getVariableCount() > 0 &&
        // getMimic() == nullptr` -- exactly `active_joint_indices` (see that
        // accessor's doc comment: "excludes fixed and mimic joints").
        let mut unbounded_variable_names = Vec::new();
        for &joint_index in group.active_joint_indices() {
            let joint = model.joint_model_at(joint_index);
            let all_found = joint
                .variable_names()
                .iter()
                .all(|name| bounds.iter().any(|b| &b.variable_name == name));
            if !all_found {
                unbounded_variable_names.extend(joint.variable_names().iter().cloned());
            }
        }

        Ok(Self {
            group,
            bounds,
            unbounded_variable_names,
        })
    }

    /// Not an upstream accessor (`bounds_.size()` is inlined at every call
    /// site there as `getConstrainedJointCount`): the number of variables
    /// with a tightened range from a [`JointConstraint`].
    pub fn constrained_variable_count(&self) -> usize {
        self.bounds.len()
    }

    /// `getUnconstrainedJointCount`, adapted to this port's flattened
    /// per-variable storage: the number of variables sampled from the
    /// joint's own bounds rather than a tightened one.
    pub fn unconstrained_variable_count(&self) -> usize {
        self.unbounded_variable_names.len()
    }
}

impl ConstraintSampler for JointConstraintSampler {
    fn joint_model_group(&self) -> &JointModelGroup {
        &self.group
    }

    fn frame_dependency(&self) -> &[String] {
        // Upstream never pushes to `frame_depends_` from this sampler: a
        // joint-space constraint has no mobile reference frame to depend on.
        &[]
    }

    fn is_joint_constraint_sampler(&self) -> bool {
        true
    }

    /// `JointConstraintSampler::sample`.
    ///
    /// # Unbounded joints: a scratch [`RobotState`] instead of a duplicated
    /// per-joint-type random draw
    ///
    /// Upstream draws each unbounded joint's own random positions via
    /// `JointModel::getVariableRandomPositions`, which knows each joint
    /// kind's own sampling rule (uniform per bounded axis, a uniformly
    /// random unit quaternion for a floating joint's rotation, etc.). This
    /// port's equivalent, [`cspace_core::state::RobotState::set_to_random_positions_with`],
    /// already implements exactly that per-joint-kind logic (verified
    /// against `{revolute,prismatic,planar,floating}_joint_model.cpp`) but
    /// keeps it private to `cspace_core::state`, and this crate has no license to
    /// reach into `cspace_core::state`'s internals or reimplement the same
    /// per-joint-kind rule a second time (`PORTING-PLAN.md`'s single-source
    /// stance). Randomizing a throwaway whole-model [`RobotState`] and
    /// reading back only this sampler's unbounded variables gets the same
    /// per-joint-kind-correct distribution for those variables through the
    /// public API alone, at the cost of drawing (and discarding) random
    /// values for every other active joint in the model too — cheap next to
    /// an IK-backed sampler's own per-attempt cost, and not on any hot path
    /// this crate's own done-criteria measure.
    fn sample(&self, state: &mut RobotState<'_>, mut rng: &mut dyn Rng) -> bool {
        if !self.unbounded_variable_names.is_empty() {
            let model = state.model();
            let mut scratch = RobotState::new(model);
            scratch.set_to_random_positions_with(&mut rng);
            for name in &self.unbounded_variable_names {
                let value = scratch.variable_position(name).expect(
                    "unbounded_variable_names only holds names resolved against this model",
                );
                state.set_variable_position(name, value).expect(
                    "unbounded_variable_names only holds names resolved against this model",
                );
            }
        }

        // Enforce the constraints for the constrained components after the
        // unbounded draw, so a variable that is both part of a
        // partially-bounded multi-variable joint (randomized above, whole
        // joint at once) and individually constrained ends up with its
        // constrained value, not its unconstrained one.
        for info in &self.bounds {
            let value = rng.random_range(info.min_bound..=info.max_bound);
            state
                .set_variable_position(&info.variable_name, value)
                .expect("bounds only holds variable names resolved against this model");
        }

        // "we are always successful"
        true
    }
}

/// Samples a group by sampling each member sampler in turn, in an order
/// that makes a sampler whose frame depends on a link another sampler sets
/// run after it.
///
/// Upstream `constraint_samplers::UnionConstraintSampler`.
pub struct UnionConstraintSampler {
    group: JointModelGroup,
    samplers: Vec<Box<dyn ConstraintSampler>>,
    frame_depends: Vec<String>,
}

impl UnionConstraintSampler {
    /// Upstream's constructor: sorts `samplers` by `order_samplers` (a
    /// stable sort, upstream's own `std::stable_sort`, so equally-ordered
    /// samplers keep their input order) and collects the sorted list's
    /// frame dependencies.
    ///
    /// `group_name` matches upstream: used only to resolve this sampler's
    /// own [`ConstraintSampler::joint_model_group`] (needed so a
    /// higher-level union-of-unions can itself be ordered by
    /// `order_samplers`); each member sampler already carries its own
    /// group.
    ///
    /// # Errors
    ///
    /// [`Error::UnknownName`] if `group_name` does not name a group in
    /// `model`.
    pub fn new(
        model: &RobotModel,
        group_name: &str,
        mut samplers: Vec<Box<dyn ConstraintSampler>>,
    ) -> Result<Self> {
        let group = model.joint_model_group(group_name)?.clone();

        samplers.sort_by(|a, b| order_samplers(a.as_ref(), b.as_ref()));

        let mut frame_depends = Vec::new();
        for sampler in &samplers {
            frame_depends.extend(sampler.frame_dependency().iter().cloned());
        }

        Ok(Self {
            group,
            samplers,
            frame_depends,
        })
    }

    /// `getSamplers`: the sorted internal list, in sampling order.
    pub fn samplers(&self) -> &[Box<dyn ConstraintSampler>] {
        &self.samplers
    }
}

impl ConstraintSampler for UnionConstraintSampler {
    fn joint_model_group(&self) -> &JointModelGroup {
        &self.group
    }

    fn frame_dependency(&self) -> &[String] {
        &self.frame_depends
    }

    /// `UnionConstraintSampler::sample`. Upstream's leading `state =
    /// reference_state;` is not reproduced — see this module's doc comment
    /// on why [`ConstraintSampler::sample`] takes no separate reference
    /// state to begin with.
    fn sample(&self, state: &mut RobotState<'_>, rng: &mut dyn Rng) -> bool {
        for sampler in &self.samplers {
            // "ConstraintSampler::sample returns states with dirty link
            // transforms (because it only writes values) but requires a
            // state with clean link transforms as input" (upstream comment,
            // union_constraint_sampler.cpp:150-152).
            let _ = state.update();
            if !sampler.sample(state, rng) {
                return false;
            }
        }
        true
    }
}

/// `OrderSamplers::operator()`. A [`Ordering::Less`] return means `a` must
/// be sampled before `b`.
fn order_samplers(a: &dyn ConstraintSampler, b: &dyn ConstraintSampler) -> Ordering {
    let a_updates: BTreeSet<&str> = a
        .joint_model_group()
        .updated_link_names()
        .iter()
        .map(String::as_str)
        .collect();
    let b_updates: BTreeSet<&str> = b
        .joint_model_group()
        .updated_link_names()
        .iter()
        .map(String::as_str)
        .collect();

    // `std::includes(a, b)`: every element of b_updates is in a_updates.
    let a_contains_b = b_updates.is_subset(&a_updates);
    let b_contains_a = a_updates.is_subset(&b_updates);

    if a_contains_b && !b_contains_a {
        return Ordering::Less;
    }
    if b_contains_a && !a_contains_b {
        return Ordering::Greater;
    }

    // Sets are equal or disjoint: fall back to frame dependency.
    let a_depends_on_b = a
        .frame_dependency()
        .iter()
        .any(|frame| b_updates.contains(frame.as_str()));
    let b_depends_on_a = b
        .frame_dependency()
        .iter()
        .any(|frame| a_updates.contains(frame.as_str()));

    if a_depends_on_b && b_depends_on_a {
        // Circular frame dependency. Upstream logs a warning and returns
        // `true` (a before b) as an arbitrary tie-break; this port has no
        // logging path to condition on (matching this crate's existing
        // `verbose`-flag D-decision — see this crate's introducing doc
        // comment), so only the tie-break itself is reproduced.
        return Ordering::Less;
    }
    if b_depends_on_a {
        return Ordering::Less;
    }
    if a_depends_on_b {
        return Ordering::Greater;
    }

    // Neither depends on the other: prefer JointConstraintSamplers, then
    // break ties alphabetically by group name.
    match (
        a.is_joint_constraint_sampler(),
        b.is_joint_constraint_sampler(),
    ) {
        (true, false) => Ordering::Less,
        (false, true) => Ordering::Greater,
        _ => a.group_name().cmp(b.group_name()),
    }
}
