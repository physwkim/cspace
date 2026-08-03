// Copyright (c) 2011, Willow Garage, Inc.
// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// Ported from moveit2 @ e017c91ee12984393a28ba246075c65f69cde3bf:
//   moveit_core/kinematic_constraints/include/moveit/kinematic_constraints/kinematic_constraint.hpp
//   moveit_core/kinematic_constraints/src/kinematic_constraint.cpp
//   moveit_core/constraint_samplers/include/moveit/constraint_samplers/constraint_sampler.hpp
//   moveit_core/constraint_samplers/include/moveit/constraint_samplers/default_constraint_samplers.hpp
//   moveit_core/constraint_samplers/src/default_constraint_samplers.cpp
//   moveit_core/constraint_samplers/include/moveit/constraint_samplers/union_constraint_sampler.hpp
//   moveit_core/constraint_samplers/src/union_constraint_sampler.cpp
//   moveit_core/constraint_samplers/include/moveit/constraint_samplers/constraint_sampler_manager.hpp
//   moveit_core/constraint_samplers/src/constraint_sampler_manager.cpp

//! Kinematic constraints and their `decide()` — [`JointConstraint`],
//! [`PositionConstraint`], [`OrientationConstraint`], [`VisibilityConstraint`]
//! and the aggregate [`KinematicConstraintSet`] — plus, in `sampler` and
//! `ik_sampler`, the `constraint_samplers` that sample states satisfying a
//! constraint rather than just evaluating one: [`ConstraintSampler`],
//! [`JointConstraintSampler`], [`UnionConstraintSampler`],
//! [`IkConstraintSampler`] and [`IkConstraintSamplerAdapter`]. See
//! `sampler`'s own doc comment for its deviations from upstream's base class
//! shape, and [`IkConstraintSampler`]'s own doc comment for why it does not
//! implement [`ConstraintSampler`] itself ([`IkConstraintSamplerAdapter`]
//! does, and is what a caller actually builds). [`select_default_sampler`],
//! in `constraint_sampler_manager`, ports `ConstraintSamplerManager::
//! selectDefaultSampler` — see that module's own doc comment for what of
//! `ConstraintSamplerManager` is and is not ported.
//!
//! # Scope
//!
//! This crate ports `kinematic_constraints/kinematic_constraint.{hpp,cpp}` —
//! the four constraint types and their `decide()`/construction logic — and,
//! in [`utils`], 13 of the 15 functions of `kinematic_constraints/utils.{hpp,cpp}`
//! that build or update a [`KinematicConstraintSet`] from plain geometric
//! arguments rather than from ROS parameters (a stale "11" stood here before
//! this doc comment's Round 6 symbol audit below counted again from the
//! header directly: `resolve_position_constraint_frame`/
//! `resolve_orientation_constraint_frame` were added after this line was
//! first written and the count was never revisited). See that module's doc
//! comment for exactly what is and is not ported from `utils.{hpp,cpp}` and
//! why.
//!
//! `equal()`, `print()`, `clear()` and `getMarkers()` are also not ported:
//! none of them are exercised by `decide()`, which is this phase's
//! completion condition (`PORTING-PLAN.md` §5 Phase 5). See the report this
//! crate's introducing commits carry for what remains `UNFIXED`.
//!
//! # No `moveit_msgs::Constraints` — and no `configure(msg, tf)`
//!
//! Upstream's four `configure()` methods each take a
//! `moveit_msgs::msg::{Joint,Position,Orientation,Visibility}Constraint`.
//! Every one of those messages encodes optionality as a `bool has_x` field
//! beside an `x` value — the same dual-meaning problem `PORTING-PLAN.md` §4.1
//! already named for `RobotState`'s dirty flags. This crate has no
//! `moveit_msgs` type to receive at all (D1), so there is no `configure()`
//! parallel to port in the first place; instead each constraint type has a
//! `new()` that takes plain Rust arguments chosen so the illegal
//! combinations upstream's message shape allows cannot be constructed here.
//! `PORTING-PLAN.md` §4.3 (renumbered under Phase 5) records the specific
//! mapping decision for each type and names the conversions a future
//! `moveit-ros::TryFrom<moveit_msgs::...>` will have to report as lossy.
//!
//! # `VisibilityConstraint` is ported in full
//!
//! Upstream's `VisibilityConstraint::decide()` builds a mesh cone between
//! sensor and target and collision-checks it against the robot via a local,
//! throwaway `collision_detection::CollisionEnvFCL`. This port does the
//! same over `moveit_collision::ParryCollisionEnv` — see
//! [`VisibilityConstraint::decide`]'s doc for why that needs no
//! `PlanningScene`/broader collision world, only `moveit-collision`
//! (already a dependency of this crate, no `moveit-scene` needed).
//!
//! # Round 6 symbol audit
//!
//! Every public symbol declared in upstream's
//! `kinematic_constraint.hpp`/`utils.hpp`, classified as `ported as <symbol>`,
//! `D-decision excludes it`, or `unported (why not yet)`. Re-run by reading
//! the two headers directly (not by inferring from what already exists here)
//! before trusting this table for a future phase's completion check.
//!
//! ## `KinematicConstraint` (abstract base) and its shared members
//!
//! - `ConstraintEvaluationResult` -> ported as [`ConstraintEvaluationResult`],
//!   direct transcription (see its own doc for why no field needed a redesign).
//! - `clear()` -> unported: no concrete type keeps mutable post-construction
//!   state to clear. A "cleared" constraint is a fresh `new()` call instead.
//! - `decide(state, verbose)` -> ported per concrete type as `decide()`; the
//!   `verbose` flag is dropped (D-decision: no logging path exists in this
//!   crate to condition on it).
//! - `enabled()` -> ported only on [`VisibilityConstraint`] (the one type that
//!   genuinely can be disabled post-construction, all three criteria `None`).
//!   `JointConstraint`/`PositionConstraint`/`OrientationConstraint` have no
//!   disabled representation and always report satisfied-by-construction
//!   (D-decision: illegal/disabled states are prevented at `new()` instead of
//!   checked at `decide()`).
//! - `equal(other, margin)` -> unported (blanket note above: not exercised by
//!   `decide()`).
//! - `getType()` -> unported directly; the equivalent information is the
//!   [`Constraint`] enum's variant tag itself (D-decision: structural, not a
//!   method).
//! - `print(out)` -> unported (blanket note above).
//! - `getConstraintWeight()` -> ported as `weight()` on each concrete type
//!   individually, not on a shared base (D-decision: no trait unites the
//!   four constraint types; each is a variant of the [`Constraint`] enum).
//! - `getRobotModel()` -> unported: no concrete type retains its constructing
//!   `&RobotModel`; construction resolves what it needs (link/variable index)
//!   and releases the borrow.
//!
//! ## `JointConstraint`
//!
//! - Two-step `ctor(model)` + `configure(msg)` -> collapsed into
//!   [`JointConstraint::new`] (see "No `moveit_msgs::Constraints`" above).
//! - `equal`, `enabled`, `clear`, `print` -> unported (see base, above).
//! - `decide` -> ported as [`JointConstraint::decide`].
//! - `getJointModel()` -> unported: no accessor keeps the upstream
//!   `JointModel*`; only the joint's variable identity is retained, and
//!   nothing in this crate or its tests needs the pointer itself.
//! - `getLocalVariableName()` -> ported as `local_variable_name()`.
//! - `getJointVariableName()` -> ported as `joint_variable_name()`.
//! - `getJointVariableIndex()` -> ported as `joint_variable_index()`.
//! - `getDesiredJointPosition()` -> ported as `desired_joint_position()`.
//! - `getJointToleranceAbove()` -> ported as `joint_tolerance_above()`.
//! - `getJointToleranceBelow()` -> ported as `joint_tolerance_below()`.
//! - File-private `normalizeAngle` (`kinematic_constraint.cpp:67-79`) ->
//!   ported as private fn `normalize_angle` in `joint.rs`, a direct
//!   transcription (distinct from the `angles`-crate `normalize_angle` used
//!   elsewhere in this workspace — upstream itself keeps two).
//!
//! ## `OrientationConstraint`
//!
//! - Two-step `ctor(model)` + `configure(msg)` -> collapsed into
//!   [`OrientationConstraint::new`].
//! - `equal`, `enabled`, `clear`, `print` -> unported (see base, above).
//! - `decide` -> ported as [`OrientationConstraint::decide`].
//! - `getLinkModel()` -> ported name-only as `link_name()`; no `LinkModel`
//!   pointer is exposed (unneeded by any caller in this crate).
//! - `getReferenceFrame()` -> ported as `reference_frame()`.
//! - `mobileReferenceFrame()` -> ported as `mobile_reference_frame()`.
//! - `getDesiredRotationMatrixInRefFrame()` -> ported as
//!   [`OrientationConstraint::desired_rotation_matrix_in_ref_frame`] (round 9
//!   — closed once `ik_sampler`'s `samplePose` `ROTATION_VECTOR` branch and
//!   its unconditional final quaternion construction needed it).
//! - `getDesiredRotationMatrix()` -> ported as
//!   [`OrientationConstraint::desired_rotation_matrix`] (round 9, same
//!   reason).
//! - `getXAxisTolerance()`/`getYAxisTolerance()`/`getZAxisTolerance()`/
//!   `getParameterizationType()` -> D-decision excludes all four as separate
//!   accessors: folded into one [`OrientationConstraint::tolerance`] accessor
//!   returning the whole [`OrientationTolerance`] enum (one sum type instead
//!   of three floats plus a discriminating tag — see that enum's own
//!   "Deviation from upstream" doc).
//! - File-private `normalizeAbsoluteAngle`, `calcEulerAngles` -> ported as
//!   private fns `normalize_absolute_angle`, `calc_euler_angles` in
//!   `orientation.rs`, direct transcriptions.
//!
//! ## `PositionConstraint`
//!
//! - Two-step `ctor(model)` + `configure(msg)` -> collapsed into
//!   [`PositionConstraint::new`].
//! - `equal`, `enabled`, `clear`, `print` -> unported (see base, above).
//! - `decide` -> ported as [`PositionConstraint::decide`].
//! - `getLinkModel()` -> ported name-only as `link_name()`, same reasoning as
//!   `OrientationConstraint::getLinkModel()` above.
//! - `getLinkOffset()` -> ported as `link_offset()`.
//! - `hasLinkOffset()` -> ported as `has_link_offset()`.
//! - `getConstraintRegions()` -> ported as `constraint_regions()`, returning
//!   `&[ConstraintRegion]` — one `Vec` of a sum type replacing upstream's four
//!   parallel vectors (`constraint_region_`, `constraint_region_pose_`, and
//!   the mesh/primitive split) — see [`ConstraintRegion`]'s own doc.
//! - `getReferenceFrame()` -> ported as `reference_frame()`.
//! - `mobileReferenceFrame()` -> ported as `mobile_reference_frame()`.
//!
//! ## `VisibilityConstraint`
//!
//! - Two-step `ctor(model)` + `configure(msg)` -> collapsed into
//!   [`VisibilityConstraint::new`].
//! - `equal`, `clear`, `print` -> unported (see base, above).
//! - `getVisibilityCone(pose, cone)` -> ported as private `cone_mesh()`
//!   (Round 4 tail disposition, folded in here so it is not rediscovered as a
//!   gap).
//! - `getMarkers(state, markers)` -> unported: no `visualization_msgs::
//!   MarkerArray` equivalent exists anywhere in this workspace (D1), and
//!   nothing in Phase 5's `decide()`-based completion condition exercises it.
//! - `enabled()` -> ported as `enabled()` — the one `enabled()` in this whole
//!   crate that is a real runtime check rather than always-true-by-
//!   construction, matching upstream's own semantics (this type alone can be
//!   constructed with all three visibility criteria absent).
//! - `decide` -> ported as [`VisibilityConstraint::decide`].
//! - Protected `decideContact(contact)` -> ported as free fn
//!   `allow_sensor_or_target_contact` in `visibility.rs` (Round 4 tail
//!   disposition, folded in here).
//!
//! ## `KinematicConstraintSet`
//!
//! - `ctor(model)` -> [`KinematicConstraintSet::new`] takes no model:
//!   constituent constraints are already resolved against a model at their
//!   own construction, so the set itself never needs to retain one
//!   (D-decision).
//! - `clear()` -> unported: no persistent resource to release beyond the
//!   `Vec`'s own `Drop`; a caller wanting an empty set constructs a fresh
//!   [`KinematicConstraintSet::new`].
//! - `add(moveit_msgs::msg::Constraints, tf)` and the four
//!   `add(vector<{Joint,Position,Orientation,Visibility}Constraint>, ...)`
//!   overloads -> D-decision excludes all five: no `moveit_msgs` type exists
//!   to add from (D1). The ported equivalent is `push(Constraint)`, adding one
//!   already-resolved constraint at a time.
//! - `decide(state, verbose)` -> ported as `decide()`.
//! - `decide(state, results, verbose)` -> ported as `decide_each()`, returning
//!   the per-constraint `Vec` directly instead of writing through an
//!   out-parameter.
//! - `equal(other, margin)`, `print(out)` -> unported (see base, above).
//! - `getPositionConstraints()`/`getOrientationConstraints()`/
//!   `getJointConstraints()`/`getVisibilityConstraints()` -> D-decision
//!   excludes all four: `constraints()` returns one flat `&[Constraint]` (a
//!   `Vec` of a sum type, not four parallel `moveit_msgs` vectors — see
//!   [`Constraint`]'s own "Deviation from upstream" doc); a caller wanting
//!   only one kind filters that slice.
//! - `getAllConstraints()` -> unported: no `moveit_msgs::msg::Constraints` to
//!   reaggregate into (D1).
//! - `empty()` -> ported as `is_empty()`.
//! - Not upstream: `len()`, `constraints_mut()`, `push()` — new API this
//!   port's reconstruct-rather-than-mutate update model needs (see
//!   `crate::utils`'s `update_*` functions, which call `constraints_mut()`).
//!
//! ## `utils.hpp` (15 declarations, 1 excluded — see [`utils`] for detail)
//!
//! - `mergeConstraints` -> ported as `merge_constraints`.
//! - `countIndividualConstraints` -> ported as `count_individual_constraints`.
//! - `constructGoalConstraints(state, jmg, tolerance)` and its two-tolerance
//!   overload -> collapsed, ported as `construct_goal_joint_constraints`.
//! - `updateJointConstraints` -> ported as `update_joint_constraints`.
//! - `constructGoalConstraints(link, pose, tol_pos, tol_angle)` (sphere
//!   region) -> ported as `construct_goal_pose_constraints`.
//! - `constructGoalConstraints(link, pose, vec tol_pos, vec tol_angle)` (box
//!   region) -> ported as `construct_goal_pose_constraints_box`.
//! - `updatePoseConstraint` -> ported as `update_pose_constraint`.
//! - `constructGoalConstraints(link, quat, tolerance)` -> ported as
//!   `construct_goal_orientation_constraints`.
//! - `updateOrientationConstraint` -> ported as `update_orientation_constraint`.
//! - `constructGoalConstraints(link, reference_point, goal_point, tolerance)`
//!   and its reference-point-omitted overload -> collapsed, ported as
//!   `construct_goal_position_constraints`.
//! - `updatePositionConstraint` -> ported as `update_position_constraint`.
//! - `constructConstraints(node, param_name, constraints)` -> D-decision
//!   excludes it: parses YAML through an `rclcpp::Node` parameter server,
//!   which this core crate has no access to at all (D1/D2) — a `moveit-ros`
//!   concern, not this crate's. Its six file-private helper functions
//!   (`collectConstraints` and the per-type `constructConstraint`/
//!   `constructPoseStamped` overloads that exist only to support it) share
//!   the same exclusion for the same reason, folded in here so they are not
//!   rediscovered as gaps.
//! - `resolveConstraintFrames` -> ported, split into
//!   `resolve_position_constraint_frame` and
//!   `resolve_orientation_constraint_frame` (`PORTING-PLAN.md` §23.1
//!   merge-time correction; see `utils.rs`'s own doc for why the split runs
//!   before construction rather than after).

mod constraint_sampler_manager;
mod ik_sampler;
mod joint;
mod orientation;
mod position;
mod sampler;
mod set;
pub mod utils;
mod visibility;

pub use constraint_sampler_manager::select_default_sampler;
pub use ik_sampler::{IkConstraintSampler, IkConstraintSamplerAdapter, IkSamplingPose};
pub use joint::JointConstraint;
pub use orientation::{OrientationConstraint, OrientationTolerance};
pub use position::{ConstraintRegion, PositionConstraint};
pub use sampler::{ConstraintSampler, JointConstraintSampler, UnionConstraintSampler};
pub use set::{Constraint, KinematicConstraintSet};
pub use visibility::{
    SensorSpec, SensorViewDirection, TargetSpec, VisibilityConstraint, VisibilityCriteria,
};

/// The result of evaluating one constraint against a state. Upstream
/// `kinematic_constraints::ConstraintEvaluationResult`.
///
/// Unlike the four constraint types themselves, this struct needed no
/// `Option`/enum redesign: both fields always hold one meaning regardless of
/// context (`satisfied` is never conditionally overloaded, and `distance` is
/// always "how far from satisfied, in the constraint's own units" —
/// `0.0` both when perfectly satisfied and, degenerately, for a disabled
/// constraint that always reports satisfied). It is ported as a direct
/// transcription.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ConstraintEvaluationResult {
    /// Whether the constraint was satisfied by the state it was evaluated
    /// against.
    pub satisfied: bool,
    /// The distance from being satisfied, weighted by the constraint's own
    /// weight. `0.0` when satisfied.
    pub distance: f64,
}

impl ConstraintEvaluationResult {
    /// Build a result. Upstream's `ConstraintEvaluationResult` constructor.
    pub fn new(satisfied: bool, distance: f64) -> Self {
        Self {
            satisfied,
            distance,
        }
    }
}
