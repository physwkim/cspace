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
//! # Completion statement
//!
//! Every number below is a command someone can re-run.
//!
//! **`kinematic_constraint.hpp`/`utils.hpp`** (the `decide()`-facing half,
//! `PORTING-PLAN.md` §5 Phase 5's own completion condition):
//! `rg -c '^//! - `' crates/moveit-constraints/src/lib.rs` is **61** — the
//! "Round 6 symbol audit" section above, including the 14 `utils.hpp`
//! bullets folded into its own final subsection (a bullet count, not a
//! 1:1 declaration count — a few bullets there fold sibling overloads
//! together, e.g. `getXAxisTolerance()`/`getYAxisTolerance()`/
//! `getZAxisTolerance()`/`getParameterizationType()` on one line). That
//! audit's own conclusion, unchanged this round: every unported symbol is
//! D1-excluded, a documented structural D-decision, or explicitly not
//! exercised by `decide()` — no undocumented gap survived it.
//!
//! **`constraint_samplers/*.hpp`** (round 12's audit, immediately below
//! this section, re-verified symbol-by-symbol against upstream's `.cpp`
//! rather than the header in round 13): `rg -c '^//! - CS:'
//! crates/moveit-constraints/src/lib.rs` is **66**, one bullet per
//! declaration exactly (no folding). 18 ported, 23 structural, 8
//! D4-excluded, 6 D1-excluded, **11 real gaps** — see that section for the
//! exact breakdown and why none of the 11 block Phase 5's `decide()`-based
//! completion condition (they are sampler-side diagnostics/benchmarking,
//! not constraint evaluation). Round 13 found one of round 12's two
//! `getGroupStateValidityCallback`/`setGroupStateValidityCallback` `gap`
//! tags did not hold up and ported it — see
//! [`IkConstraintSampler::sample`]/[`IkConstraintSamplerAdapter`].
//!
//! **Tests.** `cargo nextest run -p moveit-constraints --no-fail-fast`:
//! **92** tests, 92 passed. Of those,
//! `rg -c '^mod oracle_' crates/moveit-constraints/tests/utils_parity.rs`
//! is **7** — real moveit2-oracle comparison modules, not self-referential
//! assertions. Ground truth is the `panda_constraints` entry of
//! `tests/fixtures/oracle-models.json`, checked against the current oracle
//! image every round by `tools/ci/verify-fixture-replay.sh` — not a fixed
//! image tag pinned in prose here, which would only ever describe the image
//! that captured the fixture, not the one this crate is verified against
//! today. Together they hold **16** `#[test]` functions
//! (`sed -n '176,555p' crates/moveit-constraints/tests/utils_parity.rs | rg
//! -c '#\[test\]'`, the line range spanning from the first `mod oracle_`
//! block through the last); see `utils_parity.rs`'s own module doc for the
//! oracle image tag and what each case checks.
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
//!
//! ## `constraint_samplers/*.hpp` symbol audit (round 12)
//!
//! Unlike the audit above, this one had never been run before this round —
//! `sampler.rs`/`ik_sampler.rs`/`constraint_sampler_manager.rs` each carried
//! their own architecture doc but no symbol-by-symbol walk, so the gaps
//! tagged `gap` below were previously invisible rather than deliberately
//! deferred. Read directly from
//! `constraint_sampler{,_allocator,_manager,_tools}.hpp`,
//! `default_constraint_samplers.hpp` and `union_constraint_sampler.hpp`, one
//! bullet per declaration (constructor overloads counted individually, not
//! folded together, so the count below is exact rather than a bullet
//! count). Every bullet's disposition tag — `ported`, `structural`, `D1`,
//! `D4`, or `gap` — is the exact word immediately after `->` **on the same
//! line as the `CS:` marker**, never wrapped to a continuation line, so the
//! `rg` counts below match the prose exactly; only the free-form reasoning
//! after the tag wraps.
//!
//! ### `ConstraintSampler` (abstract base, `constraint_sampler.hpp`)
//!
//! - CS: `DEFAULT_MAX_SAMPLING_ATTEMPTS` -> ported: [`sampler::DEFAULT_MAX_SAMPLING_ATTEMPTS`].
//!   Rounds 13/14 found this an undeferred gap (evidence retained below),
//!   on the reasoning that every call site in this port's design already
//!   supplied its own concrete attempt count, so there was no
//!   omitted-argument call site for a default to ever apply to. Round 20
//!   added the first such call site anyway:
//!   `constraint_sampler_manager::select_default_sampler`'s own
//!   `max_attempts` parameter is caller-supplied in exactly the same way
//!   upstream's collapsed `sample()` overloads were, and
//!   `moveit_planners_sbp::registry::RrtConnectContext::solve` is a real
//!   production caller of it that needs a value to pass rather than
//!   inventing one — so the constant moved from "nothing to port this
//!   into" to "ported", not because the round 13/14 reasoning was wrong at
//!   the time. Prior evidence: upstream's only two uses
//!   (`constraint_sampler.hpp:171,202`) are default arguments to the two
//!   `sample()` overloads this port already collapses away (tagged
//!   `structural` above); no other production file in
//!   `moveit_core`/`moveit_planners`/`moveit_ros` references the constant
//!   at all.
//! - CS: `ConstraintSampler(scene, group_name)` (ctor) -> structural:
//!   collapsed into each concrete type's own `new()`, no base constructor to
//!   share (traits are not constructed)
//! - CS: `~ConstraintSampler()` (dtor) -> structural: no Rust equivalent
//!   needed
//! - CS: `configure(constr)` -> D1: no `moveit_msgs::Constraints` to
//!   receive; also collapsed into each concrete type's own `new()`
//! - CS: `getGroupName()` -> ported: `ConstraintSampler::group_name`
//! - CS: `getJointModelGroup()` -> ported:
//!   `ConstraintSampler::joint_model_group`
//! - CS: `getPlanningScene()` -> structural: documented in `sampler.rs`'s
//!   own "# No `PlanningScene`" section — neither ported sampler needs
//!   anything a `PlanningScene` provides beyond the model
//! - CS: `getFrameDependency()` -> ported:
//!   `ConstraintSampler::frame_dependency`
//! - CS: `getGroupStateValidityCallback()` -> structural: no accessor is
//!   ported alongside the setter below — upstream's own getter has zero
//!   callers anywhere in `moveit_core`, `moveit_planners` or `moveit_ros`
//!   (round 13 audit), so there is nothing that ever reads the callback
//!   back out once set
//! - CS: `setGroupStateValidityCallback(callback)` -> ported: round 13's
//!   audit found this one is not diagnostic-only —
//!   `default_constraint_samplers.cpp`'s `sampleHelper`/`callIK`/
//!   `samplingIkCallbackFnAdapter` show this gates real IK-solution
//!   acceptance, not diagnostics — `ompl_interface/src/detail/`
//!   `constrained_goal_sampler.cpp:135` is a real production caller. Ported
//!   as `IkConstraintSampler::sample`'s `group_state_validity_callback`
//!   parameter (reusing `moveit_kinematics::SolveOptions::solution_callback`,
//!   the same accept/reject hook, rather than inventing a new type) and
//!   `IkConstraintSamplerAdapter::set_group_state_validity_callback`, baked
//!   in at construction like `max_attempts` since `ConstraintSampler::sample`
//!   has no per-call parameter for it either
//! - CS: `sample(state)` (1-arg overload) -> structural: collapsed into
//!   `ConstraintSampler::sample`'s one signature (`sampler.rs`'s own "#
//!   `sample`'s collapsed signature" section)
//! - CS: `sample(state, max_attempts)` (2-arg overload) -> structural: same
//!   collapse
//! - CS: `sample(state, reference_state)` (2-arg overload) -> structural:
//!   same collapse
//! - CS: `sample(state, reference_state, max_attempts)` (pure virtual) -> ported:
//!   same target as the three collapsed overloads above, `ConstraintSampler::sample`
//! - CS: `isValid()` -> gap: no persisted validity flag — a sampler that
//!   fails to build returns `Err` from `new()` instead of a post-hoc query.
//!   Round 13 evidence: `is_valid_` is set once by `configure`
//!   (`default_constraint_samplers.cpp:165,291`) and checked at the top of
//!   `sampleHelper` (`:589`) purely to short-circuit a never-configured
//!   sampler — a state this port's fallible `new()` makes unreachable by
//!   construction; the only external `isValid()` callers anywhere are
//!   upstream's own `test_constraint_samplers.cpp`. Round 14 re-check: (a)
//!   yes — `if (!is_valid_) { WARN; return false; }` at `:589` is a real
//!   branch condition, the same shape `setGroupStateValidityCallback` had.
//!   The difference from that case: there the *state* (an installed
//!   callback) exists and is reachable in this port's design and was simply
//!   never wired in; here the *state* `is_valid_` discriminates (configure
//!   never having succeeded) cannot be constructed at all in this port,
//!   because `new()` returns `Result` and a `IkConstraintSampler` value
//!   only ever exists already-configured. Nothing is left to route the
//!   branch to. (b) no production caller, per Round 13's evidence above.
//!   Round 15: that unreachability was believed, not yet checked as a
//!   closed invariant — `new()` being fallible only proves *one* path is
//!   guarded, not that it's the *only* path. Anchor
//!   (`rg -n '\-> Self|\-> Result<Self>|Self \{' sampler.rs ik_sampler.rs`)
//!   finds exactly one `Self { .. }` site per sampler type
//!   (`JointConstraintSampler` sampler.rs:237, `UnionConstraintSampler`
//!   sampler.rs:369, `IkConstraintSampler` ik_sampler.rs:185,
//!   `IkConstraintSamplerAdapter` ik_sampler.rs:602), and every one sits
//!   inside that type's own fallible `new()`, wrapped in `Ok(..)` — no
//!   second constructor exists to classify. Checked and ruled out for all
//!   four types: no `#[derive(Default)]` or hand-written `impl Default`
//!   (an all-zero/empty default would skip `new()`'s checks entirely), no
//!   `serde::Deserialize` impl (would reconstruct an unchecked value from
//!   external bytes), no `pub`/`pub(crate)` field (would let struct-literal
//!   syntax build one from outside `new()`'s own function body — every
//!   field on all four types is private to its module, and neither module
//!   contains a second struct-literal site to receive such a leak even if
//!   one existed), and no `unsafe` block (no transmute/`MaybeUninit`
//!   escape hatch). `#[derive(Clone)]` on `IkConstraintSampler` is the one
//!   other value-producing path and is `through new()`, not a bypass: it
//!   can only duplicate a receiver that itself already passed `new()`, so
//!   it cannot conjure a never-configured value from nothing. Classification
//!   for all four: `through new() (ok)`, zero bypasses — the invariant is
//!   structural (unrepresentable by the type), not merely convention as
//!   Round 14 had left it
//! - CS: `getVerbose()` -> gap: no verbose/logging mode exists anywhere in
//!   this crate. Round 13 evidence: `verbose_` only ever gates an
//!   `RCLCPP_INFO`/`RCLCPP_WARN` call or is forwarded into
//!   `decide(state, verbose_)` to control *its* logging
//!   (`default_constraint_samplers.cpp:612,657,659,707`) — it never changes
//!   a `decide()`/`sample()` return value, and this crate already dropped
//!   `rclcpp` logging entirely. Round 14 re-check: (a) at `:612`/`:707`
//!   `verbose_` gates only an `RCLCPP_INFO` call, no other statement in
//!   that branch — confirmed a dead end, not merely assumed. At
//!   `:657`/`:659` it is forwarded into `PositionConstraint`/
//!   `OrientationConstraint::decide(state, verbose)`, whose own `verbose`
//!   parameter this crate's Round 6 audit (`lib.rs`'s "Round 6 symbol
//!   audit" section above) already dropped entirely as D-decision "no
//!   logging path exists"; every `decide()` oracle-parity test in this
//!   crate (`utils_parity.rs`'s `oracle_construct_goal_*`/
//!   `oracle_update_*` modules) already passes without ever threading a
//!   verbose flag through, which would not be possible if `verbose`
//!   changed `.satisfied` upstream. (b) no production caller — same file,
//!   same two call sites, both internal
//! - CS: `setVerbose(verbose)` -> gap: same
//! - CS: `getName()` -> gap: debugging-only per upstream's own doc ("for
//!   debugging purposes"); every one of the four concrete implementers
//!   below drops its own override too. Round 13 evidence: `rg -n
//!   '(sampler|iks|jcs|cs)->getName\(\)|\.getName\(\)$' moveit_core/constraint_samplers`
//!   finds zero calls on a `ConstraintSampler` instance anywhere in
//!   `constraint_sampler_manager.cpp`/`union_constraint_sampler.cpp`/
//!   `default_constraint_samplers.cpp` — every `getName()` call in those
//!   files is on a `JointModelGroup`/`LinkModel`, an unrelated method that
//!   happens to share the name; the sampler's own `getName()` is called
//!   only from `test_constraint_samplers.cpp`. Round 14 re-check: (a) `rg
//!   -n 'getName\(\)\s*=='` against the same three files finds zero
//!   comparisons on any `getName()` result — every hit is a `%s`
//!   format-string argument to `RCLCPP_DEBUG` or a name string forwarded
//!   into a constructor call, never a value a branch reads; (b) zero
//!   production callers, unchanged from Round 13
//!
//! ### `JointConstraintSampler` (`default_constraint_samplers.hpp`)
//!
//! - CS: `JointConstraintSampler(scene, group_name)` (ctor) -> structural:
//!   collapsed into `JointConstraintSampler::new`
//! - CS: `JointConstraintSampler(scene, group_name, seed)` (ctor) -> structural:
//!   no internal RNG state at all —
//!   `ConstraintSampler::sample` takes `rng: &mut dyn Rng` from the caller
//!   instead
//! - CS: `configure(constr)` -> D1: no message type
//! - CS: `configure(jc)` -> structural: collapsed into
//!   `JointConstraintSampler::new`
//! - CS: `sample(state, ks, max_attempts)` -> ported: the `ConstraintSampler`
//!   trait impl
//! - CS: `getConstrainedJointCount()` -> ported:
//!   `JointConstraintSampler::constrained_variable_count`
//! - CS: `getUnconstrainedJointCount()` -> ported:
//!   `JointConstraintSampler::unconstrained_variable_count`
//! - CS: `getName()` -> gap: see base
//!
//! ### `IKSamplingPose` (`default_constraint_samplers.hpp`)
//!
//! - CS: `IKSamplingPose()` (ctor) -> structural: a struct literal replaces
//!   every one of the seven constructor overloads on this line and the six
//!   below (`ik_sampler.rs`'s own doc comment on [`IkSamplingPose`])
//! - CS: `IKSamplingPose(pc)` (ctor) -> structural: same
//! - CS: `IKSamplingPose(oc)` (ctor) -> structural: same
//! - CS: `IKSamplingPose(pc, oc)` (ctor) -> structural: same
//! - CS: `IKSamplingPose(pc ptr)` (ctor) -> structural: same
//! - CS: `IKSamplingPose(oc ptr)` (ctor) -> structural: same
//! - CS: `IKSamplingPose(pc ptr, oc ptr)` (ctor) -> structural: same
//! - CS: `position_constraint_` -> ported:
//!   `IkSamplingPose::position_constraint`
//! - CS: `orientation_constraint_` -> ported:
//!   `IkSamplingPose::orientation_constraint`
//!
//! ### `IKConstraintSampler` (`default_constraint_samplers.hpp`)
//!
//! - CS: `IKConstraintSampler(scene, group_name)` (ctor) -> structural:
//!   collapsed into `IkConstraintSampler::new`
//! - CS: `IKConstraintSampler(scene, group_name, seed)` (ctor) -> structural:
//!   no internal RNG state, same reasoning as `JointConstraintSampler`
//! - CS: `configure(constr)` -> D1: no message type
//! - CS: `configure(sp)` -> structural: collapsed into
//!   `IkConstraintSampler::new`
//! - CS: `getIKTimeout()` -> structural: no `ik_timeout_` at all —
//!   `SolverParams::max_restarts` on the solver replaces it
//!   (`ik_sampler.rs`'s own "# Deviation from upstream: no `ik_timeout_`"
//!   section)
//! - CS: `setIKTimeout(timeout)` -> structural: same
//! - CS: `getPositionConstraint()` -> gap: no accessor exposes the sampling
//!   pose's constraints back out of a built `IkConstraintSampler`. Round 13
//!   evidence: both accessors are called only from upstream's own
//!   `test_constraint_samplers.cpp`; no production file calls either. Round
//!   14 re-check: (a) not applicable — with zero production callers there
//!   is no downstream code for the returned pointer to reach, branch or
//!   otherwise; (b) zero production callers, unchanged from Round 13
//! - CS: `getOrientationConstraint()` -> gap: same
//! - CS: `getSamplingVolume()` -> ported:
//!   `IkConstraintSampler::sampling_volume`
//! - CS: `getLinkName()` -> ported: `IkConstraintSampler::link_name`
//! - CS: `sample(state, reference_state, max_attempts)` -> ported: the
//!   inherent `IkConstraintSampler::sample` — not a `ConstraintSampler`
//!   trait impl, see this type's own "Deviation from upstream: does not
//!   implement `ConstraintSampler`" doc
//! - CS: `samplePose(pos, quat, ks, max_attempts)` -> ported:
//!   `IkConstraintSampler::sample_pose`
//! - CS: `getName()` -> gap: see base
//!
//! ### `UnionConstraintSampler` (`union_constraint_sampler.hpp`)
//!
//! - CS: `UnionConstraintSampler(scene, group_name, samplers)` (ctor) -> ported:
//!   `UnionConstraintSampler::new`
//! - CS: `getSamplers()` -> ported: `UnionConstraintSampler::samplers`
//! - CS: `configure(constr)` (no-op) -> structural: no configure step exists
//!   at all — `new` is structurally always valid, matching the no-op's own
//!   always-true semantics
//! - CS: `canService(constr)` (no-op) -> D4: exists only to serve
//!   `ConstraintSamplerManager::selectSampler`'s plugin dispatch, the same
//!   mechanism excluded below
//! - CS: `sample(state, reference_state, max_attempts)` -> ported: the
//!   `ConstraintSampler` trait impl
//! - CS: `getName()` -> gap: see base
//!
//! ### `ConstraintSamplerAllocator` (`constraint_sampler_allocator.hpp`)
//!
//! - CS: `ConstraintSamplerAllocator()` (ctor) -> D4: the whole
//!   plugin-allocator interface is excluded (D4 already excludes runtime
//!   plugin-by-string dispatch; see `constraint_sampler_manager.rs`'s own
//!   "`ConstraintSamplerManager` itself is not ported" section) — nothing in
//!   this crate implements this interface
//! - CS: `~ConstraintSamplerAllocator()` (dtor) -> D4: same
//! - CS: `alloc(scene, group_name, constr)` -> D4: same
//! - CS: `canService(scene, group_name, constr)` -> D4: same
//!
//! ### `ConstraintSamplerManager` (`constraint_sampler_manager.hpp`)
//!
//! - CS: `ConstraintSamplerManager()` (ctor) -> D4: no manager struct exists
//!   at all
//! - CS: `registerSamplerAllocator(sa)` -> D4: same exclusion
//! - CS: `selectSampler(scene, group_name, constr)` -> D4: the
//!   registry-dispatch half of the manager
//! - CS: `selectDefaultSampler(scene, group_name, constr)` -> ported: the
//!   free function [`select_default_sampler`]
//!
//! ### `constraint_sampler_tools.hpp` (free functions)
//!
//! - CS: `visualizeDistribution(sampler, reference_state, link_name, sample_count, markers)` -> D1:
//!   needs `visualization_msgs::msg::MarkerArray`
//! - CS: `visualizeDistribution(constr, scene, group, link_name, sample_count, markers)` -> D1:
//!   same, plus `moveit_msgs::Constraints`
//! - CS: `countSamplesPerSecond(sampler, reference_state)` -> gap: a
//!   benchmarking helper that takes no ROS type (unlike its sibling below)
//!   and so is not D1-excluded, just never ported. Round 13 evidence: its
//!   only caller anywhere in `moveit_core`/`moveit_planners`/`moveit_ros` is
//!   its own D1-excluded `(constr, scene, group)` sibling
//!   (`constraint_sampler_tools.cpp:68`) forwarding to it — nothing outside
//!   this file calls it. Round 14 re-check, specifically not closing this
//!   with "it's a statistic, so harmless" —
//!   `constraint_sampler_tools.cpp:65-69` shows the sibling does not
//!   inspect the returned `double` at all, only forwards it straight back
//!   out as its own return value, so the number never reaches a threshold
//!   or an `if`/assert anywhere in this repo's copy of `moveit_core`: (a)
//!   no branch, confirmed by reading the one caller rather than assuming a
//!   benchmarking helper is inert; (b) no production caller — the only
//!   caller is itself D1-excluded
//! - CS: `countSamplesPerSecond(constr, scene, group)` -> D1: takes
//!   `moveit_msgs::Constraints` and `PlanningSceneConstPtr` directly
//!
//! Reproduction: `rg -c '^//! - CS:' crates/moveit-constraints/src/lib.rs`
//! is **66** — every public declaration across the six
//! `constraint_samplers/*.hpp` headers (`constraint_sampler_tools.hpp`'s
//! free functions included, `pr2_arm_ik.hpp`/`pr2_arm_kinematics_plugin.hpp`
//! excluded: those live under `constraint_samplers/test/`, not the public
//! `include/` API surface this audit covers). Breakdown, each reproducible
//! with `rg -c '^//! - CS:.*-> TAG' crates/moveit-constraints/src/lib.rs`
//! for the given `TAG`:
//!
//! - tag `ported` (implemented, findable under a Rust name given above): 19
//! - tag `structural` (collapsed constructor/configure overloads, or an
//!   internal-state field this port's design has no use for): 23
//! - tag `D4` (the plugin-allocator/registry mechanism, already excluded
//!   workspace-wide): 8
//! - tag `D1` (a ROS message or `visualization_msgs` type this crate cannot
//!   depend on): 6
//! - tag `gap` (real, not previously documented anywhere in this crate): 10
//!   — `isValid`, `getVerbose`/`setVerbose`, `getName` (four separate
//!   declarations, one per concrete type),
//!   `getPositionConstraint`/`getOrientationConstraint` on
//!   `IkConstraintSampler`, and `countSamplesPerSecond(sampler,
//!   reference_state)`. Round 13 re-verified each against upstream's `.cpp`
//!   (not just the header) rather than relying on round 12's header-only
//!   read; each bullet above now carries that evidence inline. Round 20
//!   moved `DEFAULT_MAX_SAMPLING_ATTEMPTS` from this list to `ported` (see
//!   that bullet above) once a real caller existed for it; the other ten
//!   are not exercised by `decide()`, this phase's own completion condition
//!   (see this crate's introducing doc comment) — they are
//!   debugging/diagnostic accessors or a benchmarking helper, not sampling
//!   correctness — but they are real gaps, not deferred-on-purpose ones,
//!   and are named here rather than left to be rediscovered.
//!
//! Round 13 also re-verified `getGroupStateValidityCallback`/
//! `setGroupStateValidityCallback`, round 12's other two `gap`-tagged
//! symbols, against `default_constraint_samplers.cpp` rather than the
//! header alone, and found the setter genuinely gates IK-solution
//! acceptance (a real production caller exists in
//! `ompl_interface/src/detail/constrained_goal_sampler.cpp`) — not
//! diagnostic-only. It is now ported (see the `ConstraintSampler` section
//! above); the getter, which upstream itself never calls outside its own
//! declaration, is tagged `structural` instead of carried forward as a
//! second `gap`.
//!
//! 18 + 23 + 8 + 6 + 11 = 66.
//!
//! # Assert-relative-eq inventory (round 15, this crate's own count)
//!
//! `grep -rn 'assert_relative_eq!(' crates/moveit-constraints/ --include=*.rs
//! | grep -vE ':[0-9]+:\s*(///|//!)'` — the call pattern with its doc-comment
//! lines (`///`/`//!`) filtered back out, so a doc paragraph that names the
//! macro without invoking it (this very paragraph, once written, is one, and
//! the filter is what keeps this command self-consistent rather than
//! self-inflating) cannot count itself; the workspace has already been
//! bitten four times, §73.1/§83.3/§92/§104.1, by trusting a raw occurrence
//! count over a real macro-invocation scan. Returns **0** matches, exit
//! code 1, across `src/`, `tests/`, and `Cargo.toml`. No `epsilon`-only,
//! `max_relative`-only, both-present, or neither-present site exists in
//! this crate to classify or bisect.
//!
//! # Tolerance-floor re-check (round 16)
//!
//! `70a6b31`/§115 fixed a workspace-wide `serde_json` default-parser bug
//! (non-round-tripping f64 parsing) that put 8.1% of committed fixture
//! *literals* 1 ULP (~2.22e-16 relative) off before the fix. This crate's
//! own `assert_relative_eq!` share is already confirmed 0 (round 15,
//! above), so there is no bisectable constant of that specific shape. This
//! round re-checked every other tolerance-shaped constant in the crate —
//! `EPS`/`f64::EPSILON` weight/wraparound guards in `visibility.rs`,
//! `joint.rs`, `orientation.rs`, `position.rs`; `TOLERANCE: f64 = 1e-6` in
//! `tests/utils_parity.rs`; the ad hoc `1e-6`/`1e-9`/`1e-12` literal
//! tolerances across `tests/*.rs` — against `git log -p` for each site. No
//! commit ever adjusted any of them with reference to a measured
//! comparison floor; `TOLERANCE` in particular was introduced once, at
//! `1e-6`, and never touched since. None of these constants were chosen
//! based on the pre-fix noisy floor: the pre-fix noise ceiling was ~1 ULP
//! (~2.22e-16), four to ten orders of magnitude below the smallest
//! tolerance here (`1e-12`), so even the noisiest pre-fix reading could
//! never have driven any of these values to be loosened.

mod constraint_sampler_manager;
mod ik_sampler;
mod joint;
mod orientation;
mod position;
mod sampler;
mod set;
pub mod utils;
mod visibility;

pub use constraint_sampler_manager::{SubgroupSolver, select_default_sampler};
pub use ik_sampler::{IkConstraintSampler, IkConstraintSamplerAdapter, IkSamplingPose};
pub use joint::JointConstraint;
pub use orientation::{OrientationConstraint, OrientationTolerance};
pub use position::{ConstraintRegion, PositionConstraint};
pub use sampler::{
    ConstraintSampler, DEFAULT_MAX_SAMPLING_ATTEMPTS, JointConstraintSampler,
    UnionConstraintSampler,
};
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
