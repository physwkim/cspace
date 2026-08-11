// Copyright (c) 2011-2012, Georgia Tech Research Corporation
// Copyright (c) 2026, cspace contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// Ported from moveit2 @ e017c91ee12984393a28ba246075c65f69cde3bf:
//   moveit_core/trajectory_processing/include/moveit/trajectory_processing/time_optimal_trajectory_generation.hpp (lines 193-303)
//   moveit_core/trajectory_processing/src/time_optimal_trajectory_generation.cpp (lines 918-1312)
//
// Considered and deliberately not ported:
//   moveit_core/trajectory_processing/include/moveit/trajectory_processing/time_parameterization.hpp
//   (see the "Not ported: `TimeParameterization`" section below)

//! `trajectory_processing::TimeOptimalTrajectoryGeneration`: the
//! [`crate::trajectory::robot_trajectory::RobotTrajectory`] adapter around the
//! model-independent [`crate::trajectory::path::Path`]/[`crate::trajectory::totg_trajectory::Trajectory`]
//! core ([`crate`]'s module doc comment; `PORTING-PLAN.md`'s "out of scope"
//! note this module now supersedes).
//!
//! # Out of scope
//!
//! - `computeTimeStamps(..., const std::vector<moveit_msgs::msg::JointLimits>&, ...)`
//!   — a `moveit_msgs` conversion, out of scope per `PORTING-PLAN.md` §0's
//!   recorded D1 interpretation ("코어 크레이트는 ROS 타입을 일절 참조하지
//!   않는다. `moveit_msgs` ... 에 해당하는 것은 코어 안에서 순수 Rust
//!   타입으로 새로 정의한다.", lines 20-26) — not an independently invented
//!   call: `moveit_msgs::msg::JointLimits` is exactly the ROS message type
//!   that text names. It is also a thin wrapper that unpacks a `JointLimits`
//!   message into the same `velocity_limits`/`acceleration_limits` maps
//!   [`compute_time_stamps_with_limits`] already takes (cpp:1006-1027), so
//!   nothing behavioural is lost by skipping it — matching
//!   `ruckig_smoothing`'s precedent for the analogous overload there.
//! - The `RCLCPP_ERROR`/`RCLCPP_WARN` logging calls are not ported; this
//!   crate has no logging dependency to route them through (matching
//!   `ruckig_smoothing`'s precedent). Every upstream `RCLCPP_ERROR` +
//!   `return false` site still returns [`Error`] here.
//!
//! # Closed gap: the scaling-only overload's test path
//!
//! [`compute_time_stamps`] (the scaling-only overload) could not succeed
//! against any fixture in this workspace for two rounds, because:
//!
//! 1. `cspace_core::model`'s URDF loader never sets `acceleration_bounded`.
//!    `crates/cspace-core/src/model/joint/urdf.rs`'s `joint_bounds_from_urdf`
//!    (the sole 1-DOF-joint bounds constructor) reads only `joint.limit.
//!    velocity`; nothing in that crate ever touches `max_acceleration`/
//!    `acceleration_bounded`, which stay at `VariableBounds::default`'s
//!    `false`. This matches upstream exactly: `jointBoundsFromURDF` in
//!    `moveit_core/robot_model/src/robot_model.cpp` likewise never reads
//!    an acceleration limit from a URDF `<limit>` element, because URDF's
//!    schema has no such field. Not a defect, not this crate's — still
//!    true today, and not what closed the gap.
//! 2. Until it landed, nothing outside `cspace_core::model` could set
//!    `acceleration_bounded` programmatically either.
//!    `JointModel::set_variable_bounds_from_limits` (`model.rs`, public)
//!    could do it, but nothing handed out a `&mut JointModel` to reach it
//!    — `RobotModel`'s entire public surface after construction was
//!    `&self`-only, and `from_urdf_and_srdf` its only public constructor.
//!
//! **What closed it:** `RobotModel::joint_model_mut(&mut self, name: &str)
//! -> Result<&mut JointModel>` (`crates/cspace-core/src/model/robot_model.rs`),
//! mirroring upstream's non-`const` `RobotModel::getJointModel(const
//! std::string&)` overload (`moveit_core/robot_model/include/moveit/
//! moveit/robot_model/robot_model.hpp:146`) — the same accessor upstream's own
//! `joint_limits.yaml` loaders use to call `JointModel::setVariableBounds`
//! post-construction (`moveit/robot_model/joint_model.hpp:356/359`), since URDF and
//! `joint_limits.yaml` are two different bound sources upstream, merged
//! after model load rather than in one constructor call. Landed in
//! `cspace_core::model` (out of this crate's ownership); this crate's own change
//! was the test, not the accessor.
//!
//! `totg_robot_trajectory_scaling_only_parity.rs` now exercises
//! [`compute_time_stamps`] end to end: it reads each `panda_arm` joint's
//! current bounds via `JointModel::variable_bounds_msg`, overwrites only
//! the acceleration fields, and writes them back via
//! `set_variable_bounds_from_limits` — the same read/mutate/write shape
//! upstream's own loaders use — before calling `compute_time_stamps`
//! against a real oracle-captured numeric result.
//! `oracle.cpp`'s `totgRobotTrajectoryCase` does the analogous mutation on
//! `model_` via `JointModel::getVariableBoundsMsg`/`setVariableBounds` when
//! a case carries an `"acceleration_bounds"` field.
//!
//! [`crate::trajectory::trajectory_tools::apply_totg_time_parameterization`] wraps this
//! same overload; its own test in `trajectory_tools.rs` uses the identical
//! setup.
//!
//! # `dynamics_solver`: ported, in `cspace_core::state`
//!
//! `velocity_limits`/`acceleration_limits` above (and `oracle.cpp`'s
//! `"acceleration_bounds"` case field) are caller-supplied: nothing in this
//! crate computes a per-joint acceleration bound from anything. Upstream's
//! `moveit_core/dynamics_solver` is the package that comes closest to a
//! source for one, so a future caller of [`compute_time_stamps_with_limits`]
//! reaching for real acceleration limits is the reader this belongs in
//! front of.
//!
//! **It is already ported**, as `crate::state::dynamics::DynamicsSolver`
//! (`crates/cspace-core/src/state/dynamics.rs`): `torques`, `max_torques`,
//! `max_payload` and `payload_torques`, the Recursive Newton-Euler
//! recursion written out rather than delegated to KDL, with the ROS message
//! types replaced by `nalgebra` vectors. It is verified against the
//! oracle's own `dynamics` op (`tools/moveit-oracle/src/oracle.cpp`,
//! captured by `capture-dynamics-fixtures.py`) across four robots —
//! `crates/cspace-core/tests/dynamics_parity.rs` reads
//! `{panda,fanuc,dual_arm_panda,pr2}_dynamics.json`.
//!
//! **What it does not give this crate** is an acceleration bound.
//! `getTorques` answers the forward question — the torques a given
//! configuration, velocity and acceleration require — and
//! `getMaxPayload`/`getPayloadTorques` build a payload answer on top of it.
//! There is no `getMaxAcceleration`-shaped method anywhere in the class,
//! upstream or here. A caller wanting "the acceleration each joint can
//! sustain before its torque limit saturates" would have to invert torques
//! against `max_torques()` themselves (e.g. by bisection). Upstream does
//! not do this anywhere either: `rg -n 'dynamics_solver|DynamicsSolver'`
//! across `moveit_core` outside `dynamics_solver/` returns 6 hits, all
//! build-system/changelog noise (`tools/moveit-oracle/CMakeLists.txt:55,83,108`,
//! `CHANGELOG.rst:1165,1612,2018`) — none is a code call site into
//! `trajectory_processing` or `robot_trajectory`. The "natural producer of
//! `acceleration_bounds`" framing is an analogy between two "per-joint
//! limit" concepts, not an existing call-graph edge.
//!
//! # Not ported: `TimeParameterization`
//!
//! Upstream's `TimeOptimalTrajectoryGeneration` implements the abstract
//! base class `trajectory_processing::TimeParameterization`
//! (`time_parameterization.hpp`) — three pure-virtual `computeTimeStamps`
//! overloads matching this type's own three. No trait is ported for it, for
//! a reason specific to upstream's actual implementor set rather than to
//! the mere existence of the base class:
//!
//! - `rg -n 'public TimeParameterization'` across the entire pinned
//!   upstream checkout returns exactly one match —
//!   `TimeOptimalTrajectoryGeneration` itself. `RuckigSmoothing`
//!   (`ruckig_traj_smoothing.hpp`) does not inherit it, despite covering
//!   the same "time-parameterize a trajectory" role; its own `applySmoothing`
//!   has a different name and signature. No other type anywhere in the
//!   checkout inherits it either.
//! - No call site anywhere in upstream takes a `TimeParameterization&`,
//!   `TimeParameterizationPtr`, or holds one in a container — confirmed by
//!   grepping the symbol outside its own header and
//!   `time_optimal_trajectory_generation.hpp`. There is no `pluginlib`
//!   export macro for it, and no factory selects between implementations
//!   by name or config, unlike D4's actual motivating cases (planner
//!   backends selected at runtime).
//!
//! One implementor, zero polymorphic call sites: this is an abstraction
//! upstream declared but never exercises. D4's compile-time plugin registry
//! (`PORTING-PLAN.md` §0, trait + `linkme`) earns its cost when upstream
//! actually substitutes implementations; a `TimeParameterization` trait
//! here would have exactly one `impl` and nothing that calls through the
//! trait object instead of the concrete type — an unused trait, which is a
//! worse artifact than the documented decision not to add one.
//!
//! Separately, and regardless of the above: a faithful trait would not be
//! constructible under D1 anyway. The third pure-virtual overload takes
//! `const std::vector<moveit_msgs::msg::JointLimits>&`
//! (`time_parameterization.hpp:55-58`) — the same `moveit_msgs` type this
//! module's own inherent third overload excludes above. A trait method
//! signature is exactly as core-crate-visible as a free function's, so D1
//! excludes that method the same way; a trait with two of its three
//! required methods would not mirror the base class it claims to model.
//! This is the D1 exclusion propagating from a leaf type up into an
//! interface shape, not a new decision — named here because an interface
//! is where a partial exclusion becomes structurally visible in a way a
//! single skipped free function does not.
//!
//! # Deviations from upstream
//!
//! - **No default constructor parameters, no class.** Upstream's
//!   `TimeOptimalTrajectoryGeneration` is a class whose constructor takes
//!   `path_tolerance`/`resample_dt`/`min_angle_change` (each defaulted) and
//!   whose `computeTimeStamps` overloads default
//!   `max_velocity_scaling_factor`/`max_acceleration_scaling_factor` to
//!   `1.0`. Rust has no default parameters; [`TotgOptions`] groups all five,
//!   and its [`Default`] impl reproduces every upstream default
//!   (`path_tolerance = DEFAULT_PATH_TOLERANCE`, `resample_dt = 0.1`,
//!   `min_angle_change = 0.001`, both scaling factors `1.0`) —
//!   `TotgOptions { min_angle_change: 0.01, ..Default::default() }`
//!   reproduces a call that only overrode the constructor's third
//!   parameter. [`compute_time_stamps`]/[`compute_time_stamps_with_limits`]
//!   are free functions instead of methods, for the same reason
//!   `ruckig_smoothing` uses free functions: nothing is carried across calls
//!   that a struct would need to own.
//! - **`getVariableIndexList()`/`getVariableNames()` (a global per-DOF
//!   integer index into the whole `RobotState`) is replaced by iterating
//!   [`JointModelGroup::variable_names`] and addressing each `RobotState`
//!   variable by name.** Same substitution `ruckig_smoothing` already made,
//!   for the same reason: [`crate::model::JointModelGroup`] has no
//!   index-list-returning method in this workspace, and this crate's
//!   [`crate::state::RobotState`] accessors are name-based.
//! - **The active-joint limit vector is always built by active-joint loop
//!   position, never by `vars[active_joint_indices[idx]]` as a vector
//!   index.** This is the one place this port could not transcribe upstream
//!   literally, because upstream itself is inconsistent between its two
//!   overloads:
//!   - The scaling-only overload (cpp:924-1004) builds `max_velocity`/
//!     `max_acceleration` correctly: `for (size_t idx = 0; idx <
//!     num_active_joints; ++idx)` (cpp:954) uses the *loop counter* `idx` to
//!     write `max_velocity[idx]` (cpp:968-969), and only uses
//!     `active_joint_indices[idx]` to look up the bound
//!     (`vars[active_joint_indices[idx]]`, cpp:957) — a dense,
//!     active-joint-position-indexed vector, exactly [`TotgOptions`]'s
//!     dense semantics here.
//!   - The explicit-limits overload (cpp:1029-1135) does not: `for (const
//!     auto idx : indices)` (cpp:1062) rebinds `idx` to each *element value*
//!     of `indices` (i.e. each active variable's position in the group's
//!     *full* — active-plus-mimic — variable list, cpp:1051-1053), then
//!     writes `max_velocity[idx]` (cpp:1072, 1085, 1106, 1119) into a vector
//!     sized `indices.size()` (the active-only count, cpp:1058-1060). For
//!     any group with a mimic joint, `indices` contains a strictly
//!     increasing subsequence of `0..num_all_variables` with gaps at the
//!     mimic positions, so its largest element is `>= indices.size()`
//!     whenever any gap precedes it — `max_velocity[idx]`/
//!     `max_acceleration[idx]` is then an out-of-bounds `Eigen::VectorXd`
//!     write, undefined behaviour in C++. `panda_arm` and every group in
//!     upstream's own test suite has no mimic joint, so `indices` is exactly
//!     `0..num_active` and this is invisible there; `fixtures/pr2.srdf`'s
//!     `l_end_effector`/`r_end_effector` groups (mimic gripper fingers) do
//!     reach it. There is no undefined behaviour to port in a memory-safe
//!     language, and the surrounding code's evident intent — a dense,
//!     active-joint-count-sized vector, matching the *other* overload's own
//!     correct pattern one function up — is unambiguous, so this port uses
//!     that pattern (loop position as the vector index, `indices[idx]` only
//!     to look up the bound) uniformly for both [`compute_time_stamps`] and
//!     [`compute_time_stamps_with_limits`].
//! - **The active-vs-all-variable dimension mismatch is a typed [`Error`],
//!   not a silent truncation or a panic.** `do_time_parameterization_calculations`
//!   (cpp:1162-1271) builds waypoints over *every* group variable including
//!   mimic joints (`idx = group->getVariableIndexList()`, `num_joints =
//!   group->getVariableCount()`, cpp:1183-1184) — deliberately, since a
//!   mimic joint's position is still part of the path being time-parameterized
//!   — but is handed `max_velocity`/`max_acceleration` sized to the
//!   *active-only* variable count built by the callers above
//!   (cpp:951/1058). For a group with no mimic joints these two counts
//!   coincide; for a group with one, they do not, and upstream's
//!   `Eigen::VectorXd` indexing of the undersized limit vectors against the
//!   full-sized path is itself unchecked (another `operator[]` OOB, same
//!   root cause as the previous point). This port checks
//!   `max_velocity.len() == max_acceleration.len() ==
//!   group.variable_names().len()` before calling
//!   `crate::trajectory::totg_trajectory::Trajectory::create` and returns [`Error::other`]
//!   naming the mismatch instead, per this crate's standing "failure is a
//!   value" policy (see [`crate::trajectory::Path::create`]'s entry in the crate's own
//!   module doc). In practice this means
//!   [`compute_time_stamps`]/[`compute_time_stamps_with_limits`] reject
//!   every mimic-joint group outright; there is no upstream behaviour for a
//!   *successful* mimic-joint-group call to port, since upstream never
//!   reaches one without first hitting the undefined behaviour above.
//! - **`hasMixedJointTypes` is ported as a standalone predicate,
//!   [`has_mixed_joint_types`], but is not called from
//!   `do_time_parameterization_calculations`.** Upstream's only use of it
//!   (cpp:1176-1180) is `RCLCPP_WARN` — it never gates control flow — and
//!   this crate has no logging channel to route the diagnostic through (see
//!   the "Out of scope" note above). The predicate itself is still ported
//!   byte-for-byte and exercised directly by this module's own tests; a
//!   caller wanting the diagnostic can call it before
//!   [`compute_time_stamps`].
//! - **Velocity/acceleration bound validation differs by exact comparison
//!   operator between the two overloads, transcribed as-is.** The
//!   scaling-only overload rejects `max_velocity_ <= 0.0` but only
//!   `max_acceleration_ < 0.0` (cpp:962/983); the explicit-limits overload
//!   rejects `max_velocity_ < 0.0` (not `<=`) for its own robot-model
//!   fallback branch, and `max_acceleration_ < 0.0` for its (cpp:1079/1113).
//!   A caller-supplied entry in `velocity_limits`/`acceleration_limits` is
//!   never validated at all (cpp:1069-1074/1103-1108) — only the robot-model
//!   fallback branch is. This asymmetry looks unintentional, but "transcribe
//!   the numerics rather than rewriting them into something cleaner" applies
//!   here same as everywhere else in this crate; each comparison below cites
//!   its upstream line.
//! - **The upstream out-of-range-limit error messages sometimes name the
//!   wrong variable (`vars[idx]` instead of `vars[active_joint_indices[idx]]`,
//!   cpp:964-965), a cosmetic copy-paste bug in a value never used for
//!   anything but a log string this port drops anyway.** This port's
//!   [`Error::other`] messages name the variable actually at fault.
//! - **`resample_dt` is validated at construction for every caller outside
//!   this crate, making an invalid [`TotgOptions`] unconstructible from
//!   outside this crate (§153.1/§172).** Upstream's constructor
//!   stores `resample_dt` as-is (cpp:918-920) and later narrows
//!   `parameterized->getDuration() / resample_dt_` from `double` to `size_t`
//!   with an unchecked `static_cast` (cpp:1245) — undefined behaviour in C++
//!   outside a well-behaved input range, so there is no upstream ground
//!   truth to match there. Rust's `as usize` is always defined (saturating),
//!   which turns that same UB into two distinct silently-wrong outcomes
//!   instead: `resample_dt == 0.0` with a positive duration saturates the
//!   cast to `usize::MAX`, hanging (and then exhausting memory) in the
//!   `0..=sample_count` resample loop; `resample_dt < 0.0` casts to `0`,
//!   silently producing a one-point trajectory with no error. Neither is
//!   caught by comparing against the oracle, since the oracle has no defined
//!   answer for either input.
//!
//!   A single validating call site (rather than a validating constructor)
//!   was tried first and rejected on a later round: `resample_dt` is a
//!   `pub` field also read directly by `cspace-planning`
//!   (`response_adapters/add_time_optimal_parameterization.rs`), a crate
//!   this one does not own, and the field was believed to have enough
//!   readers there to make narrowing its type a real cross-crate blast
//!   radius (`PORTING-PLAN.md` §170). Measured, not assumed, on this round:
//!   `rg -n resample_dt` against `cspace-planning` found exactly one
//!   non-production reader — a test helper reading `TotgOptions::default()`'s
//!   field to forward it positionally into
//!   `AddTimeOptimalParameterization::new`. That is a one-line
//!   field-read-to-getter-call fix, not a blast radius, so the structural
//!   fix was authorized and done: `resample_dt` is now `pub(crate)`
//!   (not private outright — see [`TotgOptions::resample_dt`]'s field doc
//!   for why), with [`TotgOptions::with_resample_dt`] as the only way to
//!   set it from outside this crate; it validates finite-and-positive
//!   before storing. Every *production* construction site in this crate
//!   (including `totg_compute_time_stamps`'s internally-recomputed
//!   `new_resample_dt`, cpp:1147/`:586` below) goes through it — there is
//!   no remaining production path that stores an invalid `resample_dt`
//!   anywhere, but this is an audited fact about today's one production
//!   struct-literal site ([`TotgOptions::default`]), not a guarantee the
//!   type itself enforces in-crate: `pub(crate)` still lets same-crate code
//!   write a raw struct literal that skips the validator (one test does
//!   this on purpose, see
//!   `resample_dt_is_unreachable_when_waypoints_collapse_to_one_point`'s
//!   doc for why).
//!
//!   `do_time_parameterization_calculations`'s separate rejection of a
//!   resulting sample count above `MAX_RESAMPLE_SAMPLE_COUNT` (a resource
//!   bound this port adds; upstream has none) is **not** redundant with the
//!   constructor check and stays exactly where it was: it depends on
//!   `parameterized.duration()`, known only at consumption time, not at
//!   `TotgOptions` construction time. Confirmed independently load-bearing
//!   by mutation: removing the constructor-level check leaves exactly one
//!   test failing (the negative-`resample_dt` case, which produces a small
//!   negative sample count the bound alone waves through); removing the
//!   sample-count bound instead hangs the suite. **Expires** if upstream
//!   adds its own `resample_dt` validation, at which point this note should
//!   instead record whatever bound upstream chose.

use std::collections::HashMap;

use nalgebra::DVector;

use crate::error::{Error, Result};
use crate::model::JointModelGroup;

use crate::trajectory::numeric::cxx_min;
use crate::trajectory::path::{DEFAULT_PATH_TOLERANCE, Path};
use crate::trajectory::robot_trajectory::RobotTrajectory;
use crate::trajectory::totg_trajectory::Trajectory;

/// `DEFAULT_TIMESTEP`, cpp:53. This constant and
/// `trajectory::VELOCITY_SWITCHING_SCAN_STEP` port the *same* upstream
/// symbol (`DEFAULT_TIMESTEP`, defined once at cpp:53), not two
/// independently-chosen values: upstream reuses that one symbol both as the
/// scan step inside `Trajectory::getNextVelocitySwitchingPoint`
/// (`cpp:522,525,541`) and as the `time_step` argument
/// `doTimeParameterizationCalculations` passes to `Trajectory::create`
/// (`cpp:1237`). This module's own `DEFAULT_TIMESTEP` exists as a separate
/// Rust constant only because the two call sites live in different modules
/// here (`trajectory.rs` vs. this file) — not because upstream had two
/// unrelated constants to mirror.
const DEFAULT_TIMESTEP: f64 = 1e-3;

/// `DEFAULT_SCALING_FACTOR`, cpp:55.
const DEFAULT_SCALING_FACTOR: f64 = 1.0;

/// Not from upstream (which has no such bound; see this module's
/// "Deviations from upstream" note on `resample_dt` validation, §172/§153.1).
/// A defensive resource cap on the resample loop's iteration count: at 24
/// `f64`s per waypoint (a generous per-joint position/velocity/acceleration
/// estimate for even a high-DOF robot) this many waypoints is already a
/// multi-gigabyte allocation, so any legitimate `resample_dt`/duration
/// combination stays far below it.
const MAX_RESAMPLE_SAMPLE_COUNT: usize = 100_000_000;

/// The five independently-defaulted parameters upstream's constructor and
/// every `computeTimeStamps` overload take. See the module-level
/// "Deviations from upstream" note on "No default constructor parameters".
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TotgOptions {
    /// `path_tolerance`, passed to `Path::create`.
    pub path_tolerance: f64,
    /// `resample_dt`: the output trajectory's resampling period.
    /// `pub(crate)` rather than private outright, so sibling modules in
    /// this crate (e.g. `crate::trajectory::trajectory_tools`) can still use
    /// struct-update syntax against [`TotgOptions::default`] — but not
    /// `pub`: no code outside this crate can name or set this field
    /// directly, so no caller outside this crate can construct an invalid
    /// [`TotgOptions`] — that guarantee is type-level and unconditional.
    /// Inside this crate, `pub(crate)` still permits a raw struct literal
    /// that bypasses [`TotgOptions::with_resample_dt`] entirely (one test
    /// deliberately does this, see
    /// `resample_dt_is_unreachable_when_waypoints_collapse_to_one_point`'s
    /// doc); the in-crate invariant "no construction site stores an invalid
    /// value" is an audited fact about today's one production struct-literal
    /// site ([`TotgOptions::default`]'s `resample_dt: 0.1`), not something
    /// the type forbids. See [`TotgOptions::with_resample_dt`] and this
    /// module's "Deviations from upstream" note (§172/§153.1) for the rest
    /// of that argument.
    pub(crate) resample_dt: f64,
    /// `min_angle_change`: the minimum per-joint change between consecutive
    /// waypoints for the later one to be kept as distinct, rather than
    /// collapsed into the former (see
    /// `do_time_parameterization_calculations`).
    pub min_angle_change: f64,
    /// A factor in `(0, 1]` which can slow down the trajectory. Values
    /// outside that range are replaced by `DEFAULT_SCALING_FACTOR` (see
    /// `verify_scaling_factor`).
    pub max_velocity_scaling_factor: f64,
    /// A factor in `(0, 1]` which can slow down the trajectory. Values
    /// outside that range are replaced by `DEFAULT_SCALING_FACTOR` (see
    /// `verify_scaling_factor`).
    pub max_acceleration_scaling_factor: f64,
}

impl Default for TotgOptions {
    /// Matches every upstream constructor/overload default.
    fn default() -> Self {
        Self {
            path_tolerance: DEFAULT_PATH_TOLERANCE,
            resample_dt: 0.1,
            min_angle_change: 0.001,
            max_velocity_scaling_factor: DEFAULT_SCALING_FACTOR,
            max_acceleration_scaling_factor: DEFAULT_SCALING_FACTOR,
        }
    }
}

impl TotgOptions {
    /// `resample_dt`: the output trajectory's resampling period.
    pub fn resample_dt(&self) -> f64 {
        self.resample_dt
    }

    /// Sets `resample_dt`, validating it first — the only way to set this
    /// field from outside this crate (the field is `pub(crate)`, not
    /// `pub`), so no external caller can construct a [`TotgOptions`] with
    /// an invalid `resample_dt`. See [`TotgOptions::resample_dt`]'s field
    /// doc for the weaker, audited-not-type-level guarantee this leaves for
    /// in-crate callers, and this module's "Deviations from upstream" note
    /// (§172/§153.1) for why this diverges from upstream's constructor,
    /// which stores whatever it is given.
    ///
    /// # Errors
    ///
    /// [`Error`] if `resample_dt` is not finite or not positive.
    pub fn with_resample_dt(mut self, resample_dt: f64) -> Result<Self> {
        if !resample_dt.is_finite() || resample_dt <= 0.0 {
            return Err(Error::other(format!(
                "resample_dt must be finite and positive, got {resample_dt}"
            )));
        }
        self.resample_dt = resample_dt;
        Ok(self)
    }
}

/// `TimeOptimalTrajectoryGeneration::computeTimeStamps` (the scaling-only
/// overload, cpp:924-1004).
///
/// Re-parameterizes `trajectory` in place with time-optimal velocity/
/// acceleration profiles, using per-joint velocity/acceleration bounds read
/// from `trajectory`'s own `RobotModel` (via its `JointModelGroup`'s
/// *active* joint variables — see the module-level "Deviations from
/// upstream" note on why mimic joints are excluded from this lookup but not
/// from the waypoints themselves), scaled by
/// `options.max_velocity_scaling_factor`/`options.max_acceleration_scaling_factor`.
///
/// An empty trajectory is returned unmodified (cpp:928-929): there is
/// nothing to time-parameterize.
///
/// # Errors
///
/// [`Error`] if: `trajectory` has no group set; any active joint variable
/// has no velocity bound, or an acceleration bound less than zero, or a
/// velocity bound less than or equal to zero (see the module-level
/// "Deviations from upstream" note on the exact per-overload comparison);
/// the group has a mimic joint (see the module-level note on the
/// active-vs-all-variable dimension mismatch this becomes); or the
/// underlying `Path::create`/`Trajectory::create` fails.
pub fn compute_time_stamps(
    trajectory: &mut RobotTrajectory<'_>,
    options: &TotgOptions,
) -> Result<()> {
    if trajectory.is_empty() {
        return Ok(());
    }

    let group = validate_group(trajectory)?;
    let velocity_scaling_factor = verify_scaling_factor(options.max_velocity_scaling_factor);
    let acceleration_scaling_factor =
        verify_scaling_factor(options.max_acceleration_scaling_factor);

    let active_variables = active_joint_variables(trajectory, group);
    let num_active = active_variables.len();
    let mut max_velocity = DVector::zeros(num_active);
    let mut max_acceleration = DVector::zeros(num_active);

    for (idx, (_, joint_name, bounds)) in active_variables.iter().enumerate() {
        if bounds.velocity_bounded {
            if bounds.max_velocity <= 0.0 {
                return Err(Error::other(format!(
                    "invalid max_velocity {} specified for '{joint_name}', must be greater than 0.0",
                    bounds.max_velocity
                )));
            }
            max_velocity[idx] = cxx_min(bounds.max_velocity.abs(), bounds.min_velocity.abs())
                * velocity_scaling_factor;
        } else {
            return Err(Error::other(format!(
                "no velocity limit was defined for joint '{joint_name}'! you have to define \
                 velocity limits in the URDF or joint_limits.yaml"
            )));
        }

        if bounds.acceleration_bounded {
            if bounds.max_acceleration < 0.0 {
                return Err(Error::other(format!(
                    "invalid max_acceleration {} specified for '{joint_name}', must be greater \
                     than 0.0",
                    bounds.max_acceleration
                )));
            }
            max_acceleration[idx] =
                cxx_min(bounds.max_acceleration.abs(), bounds.min_acceleration.abs())
                    * acceleration_scaling_factor;
        } else {
            return Err(Error::other(format!(
                "no acceleration limit was defined for joint '{joint_name}'! you have to define \
                 acceleration limits in the URDF or joint_limits.yaml"
            )));
        }
    }

    do_time_parameterization_calculations(trajectory, &max_velocity, &max_acceleration, options)
}

/// `TimeOptimalTrajectoryGeneration::computeTimeStamps` (the explicit
/// per-joint limits overload, cpp:1029-1135).
///
/// Like [`compute_time_stamps`], but every active joint variable named in
/// `velocity_limits`/`acceleration_limits` overrides the corresponding
/// `RobotModel` bound (still scaled by
/// `options.max_velocity_scaling_factor`/`options.max_acceleration_scaling_factor`).
/// A variable not named in a given map falls back to its `RobotModel`
/// bound, if any.
///
/// # Errors
///
/// Same as [`compute_time_stamps`], except a variable named in
/// `velocity_limits`/`acceleration_limits` is never bounds-checked (matching
/// upstream — see the module-level "Deviations from upstream" note) and so
/// can never fail on that variable's account.
pub fn compute_time_stamps_with_limits(
    trajectory: &mut RobotTrajectory<'_>,
    velocity_limits: &HashMap<String, f64>,
    acceleration_limits: &HashMap<String, f64>,
    options: &TotgOptions,
) -> Result<()> {
    if trajectory.is_empty() {
        return Ok(());
    }

    let group = validate_group(trajectory)?;
    let velocity_scaling_factor = verify_scaling_factor(options.max_velocity_scaling_factor);
    let acceleration_scaling_factor =
        verify_scaling_factor(options.max_acceleration_scaling_factor);

    let active_variables = active_joint_variables(trajectory, group);
    let num_active = active_variables.len();
    let mut max_velocity = DVector::zeros(num_active);
    let mut max_acceleration = DVector::zeros(num_active);

    for (idx, (variable_name, joint_name, bounds)) in active_variables.iter().enumerate() {
        let mut velocity_set = false;
        if let Some(&limit) = velocity_limits.get(variable_name) {
            max_velocity[idx] = limit * velocity_scaling_factor;
            velocity_set = true;
        }
        if bounds.velocity_bounded && !velocity_set {
            if bounds.max_velocity < 0.0 {
                return Err(Error::other(format!(
                    "invalid max_velocity {} specified for '{joint_name}', must be greater than 0.0",
                    bounds.max_velocity
                )));
            }
            max_velocity[idx] = cxx_min(bounds.max_velocity.abs(), bounds.min_velocity.abs())
                * velocity_scaling_factor;
            velocity_set = true;
        }
        if !velocity_set {
            return Err(Error::other(format!(
                "no velocity limit was defined for joint '{joint_name}'! you have to define \
                 velocity limits in the URDF or joint_limits.yaml"
            )));
        }

        let mut acceleration_set = false;
        if let Some(&limit) = acceleration_limits.get(variable_name) {
            max_acceleration[idx] = limit * acceleration_scaling_factor;
            acceleration_set = true;
        }
        if bounds.acceleration_bounded && !acceleration_set {
            if bounds.max_acceleration < 0.0 {
                return Err(Error::other(format!(
                    "invalid max_acceleration {} specified for '{joint_name}', must be greater \
                     than 0.0",
                    bounds.max_acceleration
                )));
            }
            max_acceleration[idx] =
                cxx_min(bounds.max_acceleration.abs(), bounds.min_acceleration.abs())
                    * acceleration_scaling_factor;
            acceleration_set = true;
        }
        if !acceleration_set {
            return Err(Error::other(format!(
                "no acceleration limit was defined for joint '{joint_name}'! you have to define \
                 acceleration limits in the URDF or joint_limits.yaml"
            )));
        }
    }

    do_time_parameterization_calculations(trajectory, &max_velocity, &max_acceleration, options)
}

/// `totgComputeTimeStamps` (cpp:1137-1160): resample `trajectory` to
/// (approximately) `num_waypoints` waypoints, by running
/// [`compute_time_stamps`] once to find the optimal duration, then again
/// with `resample_dt = duration / (num_waypoints - 1)`.
///
/// # Errors
///
/// [`Error`] if `num_waypoints < 2` (cpp:1147-1151), or if either
/// [`compute_time_stamps`] call fails.
///
/// # Deviation from upstream
///
/// Upstream's first `computeTimeStamps` call (cpp:1154) discards its `bool`
/// return value entirely — a failure there is silently ignored, and
/// `trajectory.getDuration()` (whatever it was before, likely `0.0` for a
/// freshly-built trajectory) is used as `optimal_duration` regardless. This
/// port propagates that first call's `Err` instead, since silently
/// continuing with a bogus `optimal_duration` (and therefore a bogus
/// `new_resample_dt`) is not a behaviour worth reproducing: every caller in
/// upstream's own test suite only ever observes the second call's result.
pub fn totg_compute_time_stamps(
    num_waypoints: usize,
    trajectory: &mut RobotTrajectory<'_>,
    max_velocity_scaling_factor: f64,
    max_acceleration_scaling_factor: f64,
) -> Result<()> {
    if num_waypoints < 2 {
        return Err(Error::other(
            "computeTimeStamps() requires num_waypoints > 1",
        ));
    }

    // `resample_dt` here is always the crate default (`0.1`), already valid
    // — `..Default::default()` inherits it without naming it, so this needs
    // no `with_resample_dt` call.
    let default_options = TotgOptions {
        max_velocity_scaling_factor,
        max_acceleration_scaling_factor,
        ..Default::default()
    };
    compute_time_stamps(trajectory, &default_options)?;
    let optimal_duration = trajectory.duration();
    let new_resample_dt = optimal_duration / (num_waypoints - 1) as f64;

    let base_options = TotgOptions {
        max_velocity_scaling_factor,
        max_acceleration_scaling_factor,
        ..Default::default()
    };
    // `new_resample_dt` can be exactly `0.0` here: `optimal_duration` is
    // `0.0` precisely when the first `compute_time_stamps` call above
    // already collapsed `trajectory` to a single waypoint (see
    // `resample_dt_is_unreachable_when_waypoints_collapse_to_one_point`'s
    // doc comment on `do_time_parameterization_calculations` for the exact
    // mechanism). In that case the second `compute_time_stamps` call below
    // will see that same collapsed trajectory and take the identical early
    // return *again*, before `resample_dt` is ever read — matching
    // upstream's own doubled early-return structure at `cpp:1219-1226`
    // (verified directly; see
    // `totg_compute_time_stamps_silently_collapses_duplicate_waypoints_matching_upstream`).
    // Any valid `resample_dt` is equivalent on that path since it is
    // provably unread, so falling back to `base_options`'s already-valid
    // default here preserves that upstream-matching `Ok` result instead of
    // introducing an `Err` upstream's own degenerate-input contract does
    // not have.
    let resample_options = if new_resample_dt.is_finite() && new_resample_dt > 0.0 {
        base_options
            .with_resample_dt(new_resample_dt)
            .expect("just checked new_resample_dt.is_finite() && new_resample_dt > 0.0")
    } else {
        base_options
    };
    compute_time_stamps(trajectory, &resample_options)
}

/// `validateGroup`-equivalent (this specific check is inlined at the top of
/// each upstream `computeTimeStamps` overload, e.g. cpp:931-936).
fn validate_group<'g>(trajectory: &RobotTrajectory<'g>) -> Result<&'g JointModelGroup> {
    trajectory.group().ok_or_else(|| {
        Error::other("it looks like the planner did not set the group the plan was computed for")
    })
}

/// `computeJointVariableIndices(group->getActiveJointModelNames(), ...)`
/// (cpp:945-946/1053), expanded into `(variable_name, joint_name, bounds)`
/// triples in active-joint order — the dense, active-joint-position-indexed
/// list both `computeTimeStamps` overloads build their `max_velocity`/
/// `max_acceleration` vectors over. See the module-level "Deviations from
/// upstream" note on why this port uses this same dense construction for
/// both overloads.
fn active_joint_variables<'s>(
    trajectory: &RobotTrajectory<'s>,
    group: &JointModelGroup,
) -> Vec<(String, String, crate::model::joint::VariableBounds)> {
    let model = trajectory.robot_model();
    let mut result = Vec::new();
    for &joint_index in group.active_joint_indices() {
        let joint = model.joint_model_at(joint_index);
        for variable_name in joint.variable_names() {
            let bounds = *joint
                .variable_bounds_for(variable_name)
                .expect("variable_name came from this joint's own variable_names()");
            result.push((variable_name.clone(), joint.name().to_string(), bounds));
        }
    }
    result
}

/// `verifyScalingFactor` (cpp:1290-1312). See the module-level "Out of
/// scope" note on why the `RCLCPP_WARN` this replaces (naming which limit
/// type — velocity or acceleration — was invalid) is not ported.
fn verify_scaling_factor(requested_scaling_factor: f64) -> f64 {
    if requested_scaling_factor > 0.0 && requested_scaling_factor <= 1.0 {
        requested_scaling_factor
    } else {
        DEFAULT_SCALING_FACTOR
    }
}

/// `hasMixedJointTypes` (cpp:1273-1288). See the module-level "Deviations
/// from upstream" note on why this is not called from
/// `do_time_parameterization_calculations`.
pub fn has_mixed_joint_types(trajectory: &RobotTrajectory<'_>, group: &JointModelGroup) -> bool {
    let model = trajectory.robot_model();
    let mut have_prismatic = false;
    let mut have_revolute = false;
    for &joint_index in group.active_joint_indices() {
        let joint = model.joint_model_at(joint_index);
        match joint.joint_type() {
            crate::model::joint::JointType::Prismatic => have_prismatic = true,
            crate::model::joint::JointType::Revolute => have_revolute = true,
            _ => {}
        }
    }
    have_prismatic && have_revolute
}

/// `doTimeParameterizationCalculations` (cpp:1162-1271).
fn do_time_parameterization_calculations(
    trajectory: &mut RobotTrajectory<'_>,
    max_velocity: &DVector<f64>,
    max_acceleration: &DVector<f64>,
    options: &TotgOptions,
) -> Result<()> {
    // This lib does not actually work properly when angles wrap around, so we need to unwind
    // the path first.
    trajectory.unwind();

    let group = validate_group(trajectory)?;
    let variable_names = group.variable_names().to_vec();
    let num_joints = variable_names.len();

    // `max_acceleration.len() != num_joints` used to be a second `||`
    // operand here. `do_time_parameterization_calculations` is a private
    // fn with exactly two call sites in this file (`compute_time_stamps`,
    // `compute_time_stamps_with_limits`), and both construct
    // `max_velocity`/`max_acceleration` as `DVector::zeros(num_active)`
    // from the same `num_active` binding for both vectors — so the two
    // lengths can never independently differ; that operand was dead
    // (bite-confirmed: neutralizing it alone left every test green).
    if max_velocity.len() != num_joints {
        return Err(Error::other(format!(
            "max_velocity/max_acceleration have {}/{} entries but the group '{}' has {num_joints} \
             variables (including any mimic joints); computeTimeStamps only builds limits for \
             active joint variables, so a group with a mimic joint cannot be time-parameterized \
             (see this module's doc comment, \"the active-vs-all-variable dimension mismatch\")",
            max_velocity.len(),
            max_acceleration.len(),
            group.name(),
        )));
    }

    let num_points = trajectory.way_point_count();

    // Have to convert into Eigen(-equivalent) data structs and remove repeated points
    // (https://github.com/tobiaskunz/trajectories/issues/3)
    let mut points: Vec<DVector<f64>> = Vec::new();
    for p in 0..num_points {
        let waypoint = trajectory.way_point(p)?;
        let mut new_point = DVector::zeros(num_joints);
        // The first point should always be kept.
        let mut diverse_point = p == 0;

        for (j, name) in variable_names.iter().enumerate() {
            new_point[j] = waypoint.variable_position(name)?;
            // If any joint angle is different, it's a unique waypoint.
            if p > 0
                && (new_point[j] - points.last().expect("p > 0 implies a prior point")[j]).abs()
                    > options.min_angle_change
            {
                diverse_point = true;
            }
        }

        if diverse_point {
            points.push(new_point);
        } else if p == num_points - 1 {
            // If the last point is not a diverse_point we replace the last added point with it
            // to make sure to always have the input end point as the last point.
            let last = points.len() - 1;
            points[last] = new_point;
        }
    }

    // Return trajectory with only the first waypoint if there are not multiple diverse points.
    if points.len() == 1 {
        let model = trajectory.robot_model();
        let mut waypoint = trajectory.way_point(0)?.clone();
        waypoint.set_variable_velocities(&vec![0.0; model.variable_count()]);
        waypoint.set_variable_accelerations(&vec![0.0; model.variable_count()]);
        trajectory.clear();
        trajectory.add_suffix_way_point(waypoint, 0.0)?;
        return Ok(());
    }

    // Now actually call the algorithm.
    let path = Path::create(&points, options.path_tolerance)?;
    let parameterized = Trajectory::create(path, max_velocity, max_acceleration, DEFAULT_TIMESTEP)?;

    // Compute sample count. `options.resample_dt` is always finite and
    // positive here — [`TotgOptions::with_resample_dt`] is the only way to
    // set it, so there is no remaining path that reaches this point with an
    // invalid value; no re-check needed. What upstream's unchecked
    // `static_cast<size_t>` does not do, and this port still must, is bound
    // the resulting sample count.
    //
    // `raw_sample_count > MAX_RESAMPLE_SAMPLE_COUNT as f64` alone (no
    // `!raw_sample_count.is_finite()` alongside it) is enough, and that is
    // not an oversight: `+inf > (any finite bound)` is `true`, so the `>`
    // comparison already catches `+inf` on its own; a `!is_finite()` guard
    // would only still be pulling weight against a NaN `raw_sample_count`,
    // and that can no longer happen here. `raw_sample_count` is
    // `parameterized.duration() / options.resample_dt`; `resample_dt` is
    // always finite and positive (above), and a finite value divided by a
    // finite positive one is never NaN (it can overflow to `+inf`, which
    // the `>` comparison already catches, but 0/0 or inf-inf is the only
    // route to NaN and neither operand can be `0.0`/non-finite here) —
    // so this is NaN only if `duration()` itself is. `duration()` can only
    // be NaN via `Trajectory::create`'s timing loop (the sole production
    // site that assigns `TrajectoryStep::time`; see `trajectory.rs`), and
    // that loop's only remaining NaN-producing case (`totg-timing-zero-
    // velocity-division`'s zero-length-segment `0.0 / 0.0`, left
    // deliberately unguarded — see the deviation documented on that loop)
    // requires two adjacent steps at an identical, unmoving position,
    // which in turn requires the whole path to be zero-length — and
    // `points.len() == 1` above already returns before `Trajectory::create`
    // is ever called for that case (independently pinned, both by
    // reasoning and by a direct reproduction, in
    // `resample_dt_is_unreachable_when_waypoints_collapse_to_one_point`'s
    // doc comment). Bite-confirmed: removing
    // `!raw_sample_count.is_finite() ||` here does not turn any test in
    // this crate's suite red (`cargo nextest run -p cspace-core`,
    // 115/115 still pass) — the clause it used to need
    // (`resample_dt_over_a_nan_duration_is_rejected`, since renamed to
    // `resample_dt_over_an_infinite_time_construction_is_rejected`) is now
    // caught earlier, by `Trajectory::create` itself.
    let raw_sample_count = (parameterized.duration() / options.resample_dt).ceil();
    if raw_sample_count > MAX_RESAMPLE_SAMPLE_COUNT as f64 {
        return Err(Error::other(format!(
            "resample_dt {} over duration {} would require {raw_sample_count} samples, \
             exceeding the {MAX_RESAMPLE_SAMPLE_COUNT} limit",
            options.resample_dt,
            parameterized.duration(),
        )));
    }
    let sample_count = raw_sample_count as usize;

    // Resample and fill in trajectory.
    let mut waypoint = trajectory.way_point(0)?.clone();
    trajectory.clear();
    let mut last_t = 0.0;
    for sample in 0..=sample_count {
        // Always sample the end of the trajectory as well. `cxx_min`, not
        // `f64::min`, matches upstream cpp:1252's `std::min(...)` (operand
        // order swapped to match too) — for fidelity/uniformity, not
        // because a NaN can reach here: the comment above already pins
        // `duration()` non-NaN by this point, and `sample as f64 *
        // options.resample_dt` is a finite non-negative product of two
        // finite non-negative operands, so neither argument to this
        // `cxx_min` call can be NaN in production.
        let t = cxx_min(
            parameterized.duration(),
            sample as f64 * options.resample_dt,
        );
        let position = parameterized.position(t);
        let velocity = parameterized.velocity(t);
        let acceleration = parameterized.acceleration(t);

        for (j, name) in variable_names.iter().enumerate() {
            waypoint.set_variable_position(name, position[j])?;
            waypoint.set_variable_velocity(name, velocity[j])?;
            waypoint.set_variable_acceleration(name, acceleration[j])?;
        }

        trajectory.add_suffix_way_point(waypoint.clone(), t - last_t)?;
        last_t = t;
    }

    Ok(())
}

#[cfg(test)]
mod tests {
    use std::collections::HashMap;
    use std::fs;

    use crate::model::{MeshSearchPaths, RobotModel};
    use crate::srdf::SrdfModel;
    use crate::state::RobotState;

    use super::*;

    fn fixture_path(file_name: &str) -> String {
        format!(
            concat!(env!("CARGO_MANIFEST_DIR"), "/../../fixtures/{}"),
            file_name
        )
    }

    fn load(urdf_name: &str, srdf_name: &str) -> RobotModel {
        let urdf_path = fixture_path(urdf_name);
        let srdf_path = fixture_path(srdf_name);
        let urdf_xml =
            fs::read_to_string(&urdf_path).unwrap_or_else(|e| panic!("read {urdf_path}: {e}"));
        let urdf = urdf_rs::read_file(&urdf_path).expect("fixture URDF must parse");
        let srdf = SrdfModel::parse_file(&srdf_path).expect("fixture SRDF must parse");
        RobotModel::from_urdf_and_srdf(&urdf, &urdf_xml, &srdf, &MeshSearchPaths::none())
            .expect("fixture model must build")
    }

    fn panda() -> RobotModel {
        load("panda.urdf", "panda.srdf")
    }

    fn pr2() -> RobotModel {
        load("pr2.urdf", "pr2.srdf")
    }

    const PANDA_ARM_JOINTS: [&str; 7] = [
        "panda_joint1",
        "panda_joint2",
        "panda_joint3",
        "panda_joint4",
        "panda_joint5",
        "panda_joint6",
        "panda_joint7",
    ];

    fn add_panda_arm_waypoint<'m>(
        trajectory: &mut RobotTrajectory<'m>,
        model: &'m RobotModel,
        positions: [f64; 7],
        dt: f64,
    ) {
        let mut state = RobotState::new(model);
        state.set_to_default_values();
        for (name, value) in PANDA_ARM_JOINTS.iter().zip(positions) {
            state.set_variable_position(name, value).unwrap();
        }
        // Upstream's own `add_waypoint` lambda passes `delta_t` for every waypoint,
        // including the first — upstream's `RobotTrajectory` never enforces that
        // `duration_from_previous[0] == 0.0` (see `robot_trajectory.rs`'s own
        // module doc comment on that invariant, added by this port). Passing `dt`
        // for an empty trajectory here would trip that invariant instead of
        // reproducing upstream's test, so the first waypoint always uses `0.0`.
        let dt = if trajectory.is_empty() { 0.0 } else { dt };
        trajectory.add_suffix_way_point(state, dt).unwrap();
    }

    fn panda_arm_limits(value_per_joint: [f64; 7]) -> HashMap<String, f64> {
        PANDA_ARM_JOINTS
            .iter()
            .zip(value_per_joint)
            .map(|(name, value)| ((*name).to_string(), value))
            .collect()
    }

    /// Upstream `testCustomLimits` (test file lines 209-240): explicit
    /// per-joint velocity/acceleration limits succeed against real
    /// `panda_arm` fixture data, the same way upstream's own
    /// `setAccelerationLimits` test helper does it — by mutating
    /// `JointModel` bounds after construction, rather than depending on the
    /// URDF loader ever setting `acceleration_bounded` (it never does; see
    /// this module's own doc, "`dynamics_solver`: ported, in
    /// `cspace_core::state`" section, point 1). Unlike that section's history,
    /// this call site never needed the mutation API to close a gap: it
    /// takes explicit `max_velocity`/`max_acceleration` vectors directly
    /// (`compute_time_stamps_with_limits`) rather than reading them off the
    /// model, so `RobotModel::joint_model_mut` is unused here — this test
    /// simply mirrors upstream's own test structure.
    #[test]
    fn upstream_test_custom_limits() {
        let model = panda();
        let group = model.joint_model_group("panda_arm").unwrap();
        let mut trajectory = RobotTrajectory::for_group(&model, Some(group));
        add_panda_arm_waypoint(
            &mut trajectory,
            &model,
            [-0.5, -3.52, 1.35, -2.51, -0.88, 0.63, 0.0],
            0.1,
        );
        add_panda_arm_waypoint(
            &mut trajectory,
            &model,
            [-0.45, -3.2, 1.2, -2.4, -0.8, 0.6, 0.0],
            0.1,
        );

        let limits = panda_arm_limits([1.3, 2.3, 3.3, 4.3, 5.3, 6.3, 7.3]);
        let result = compute_time_stamps_with_limits(
            &mut trajectory,
            &limits,
            &limits,
            &TotgOptions::default(),
        );
        assert!(result.is_ok(), "failed to compute time stamps: {result:?}");
    }

    /// `max_velocity[idx] = bounds.max_velocity.abs().min(bounds.min_velocity.abs())`
    /// used a plain `.min()` where upstream calls `std::min(fabs(max_velocity_),
    /// fabs(min_velocity_))`. `f64::min` discards a NaN wherever it sits;
    /// `std::min`/`cxx_min` return a NaN **first** (receiver) operand instead
    /// of discarding it — so a corrupted `max_velocity` bound used to be
    /// silently replaced by `min_velocity.abs()` instead of propagating.
    ///
    /// That substitution is invisible whenever `min_velocity == -max_velocity`
    /// (true for every URDF-derived bound in this crate — see
    /// `VariableBounds::max_velocity`'s own doc comment: the loader only
    /// ever stores one magnitude and treats the range as symmetric), because
    /// then `min_velocity.abs()` just reconstructs the original, uncorrupted
    /// number by coincidence. This test breaks that symmetry on purpose —
    /// `min_velocity` is set far away from `-max_velocity` — so the
    /// substituted value is observably wrong instead of accidentally right.
    ///
    /// Downstream (`Trajectory::get_min_max_path_velocity`,
    /// `trajectory.rs:841`), a genuine NaN in `self.max_velocity[i]` is
    /// itself correctly *discarded* by that line's own (pre-existing,
    /// already-correct) `cxx_min` call — NaN there is the accumulator's
    /// second operand, and `cxx_min` drops a NaN second operand exactly as
    /// `std::min` does. So the fixed behavior is not "NaN poisons the whole
    /// trajectory"; it is "the corrupted joint stops constraining velocity
    /// at all, matching what an absent limit would do" — which is what
    /// upstream's own `std::min` chain does too. Demonstrate the fix by that
    /// duration, not by `Err`/`NaN`: pin `min_velocity` to a value so tiny
    /// that, *if* it were substituted in for the corrupted `max_velocity`,
    /// it would dominate every other joint and balloon the duration; the
    /// fixed code must instead produce a duration indistinguishable from an
    /// uncorrupted baseline, because the corrupted joint drops out of the
    /// constraint set entirely.
    #[test]
    fn a_nan_max_velocity_bound_is_not_silently_replaced_by_min_velocity() {
        let model = panda();
        let mut trajectory = two_waypoint_trajectory(&model);
        // Explicit acceleration limits for every joint, so this exercises
        // the velocity `bounds` fallback (no entry in `velocity_limits`)
        // without also tripping the unrelated "no acceleration limit was
        // defined" error panda_joint1's URDF-only bounds would otherwise
        // hit first (the URDF loader never sets `acceleration_bounded`).
        let acceleration_limits = panda_arm_limits([1.3, 2.3, 3.3, 4.3, 5.3, 6.3, 7.3]);
        let baseline_result = compute_time_stamps_with_limits(
            &mut trajectory,
            &HashMap::new(),
            &acceleration_limits,
            &TotgOptions::default(),
        );
        assert!(
            baseline_result.is_ok(),
            "uncorrupted baseline must succeed: {baseline_result:?}"
        );
        let baseline_duration = trajectory.duration();

        let mut model = panda();
        let name = "panda_joint1";
        let original = model.joint_model(name).unwrap().variable_bounds()[0];
        assert!(
            original.velocity_bounded,
            "{name} must have a velocity limit for this fixture to mean anything"
        );
        assert_eq!(
            original.min_velocity, -original.max_velocity,
            "fixture premise: URDF-derived bounds are symmetric, which is exactly \
             what makes the substitution bug invisible without this test's asymmetric \
             corruption"
        );
        model
            .joint_model_mut(name)
            .unwrap()
            .set_variable_bounds(
                name,
                crate::model::joint::VariableBounds {
                    max_velocity: f64::NAN,
                    // Far from `-max_velocity`: if `.min()` substitutes this
                    // in for the corrupted `max_velocity`, panda_joint1 would
                    // be limited to crawling at 1e-6 rad/s, dominating every
                    // other joint's (real, ~1-3 rad/s) limit and ballooning
                    // the duration by orders of magnitude.
                    min_velocity: -1e-6,
                    ..original
                },
            )
            .unwrap();

        let mut trajectory = two_waypoint_trajectory(&model);
        let result = compute_time_stamps_with_limits(
            &mut trajectory,
            &HashMap::new(),
            &acceleration_limits,
            &TotgOptions::default(),
        );
        assert!(
            result.is_ok(),
            "corrupted case must still succeed: {result:?}"
        );
        let corrupted_duration = trajectory.duration();

        assert!(
            (corrupted_duration - baseline_duration).abs() < 1e-6,
            "a NaN max_velocity bound must not be silently replaced by min_velocity \
             (here, an unrelated 1e-6 rad/s crawl limit) — the corrupted joint must \
             drop out of the velocity constraint entirely, leaving duration \
             unchanged from the uncorrupted baseline: baseline {baseline_duration}, \
             corrupted {corrupted_duration}"
        );
    }

    /// Same defect, same fix, the `max_acceleration[idx]` sibling site
    /// (`bounds.max_acceleration.abs().min(bounds.min_acceleration.abs())`,
    /// `cxx_min` after the fix). `VariableBounds::max_acceleration`'s own
    /// doc comment states the same URDF-derived symmetry as
    /// `max_velocity`'s, so this test breaks it the same way, and for the
    /// same reason: without that, `min_acceleration.abs()` would silently
    /// reconstruct the correct value and the substitution bug would stay
    /// invisible. Unlike the velocity test, the fallback-to-`bounds` branch
    /// has to be forced open here — the URDF loader never sets
    /// `acceleration_bounded` (see `upstream_test_custom_limits`'s doc
    /// comment above), so `panda_joint1` is left out of `acceleration_limits`
    /// and its bounds are mutated directly to `acceleration_bounded: true`.
    #[test]
    fn a_nan_max_acceleration_bound_is_not_silently_replaced_by_min_acceleration() {
        let model = panda();
        let mut trajectory = two_waypoint_trajectory(&model);
        let acceleration_limits = panda_arm_limits([1.3, 2.3, 3.3, 4.3, 5.3, 6.3, 7.3]);
        let baseline_result = compute_time_stamps_with_limits(
            &mut trajectory,
            &HashMap::new(),
            &acceleration_limits,
            &TotgOptions::default(),
        );
        assert!(
            baseline_result.is_ok(),
            "uncorrupted baseline must succeed: {baseline_result:?}"
        );
        let baseline_duration = trajectory.duration();

        let mut model = panda();
        let name = "panda_joint1";
        let original = model.joint_model(name).unwrap().variable_bounds()[0];
        assert_eq!(
            original.min_acceleration, -original.max_acceleration,
            "fixture premise: URDF-derived bounds are symmetric, which is exactly \
             what makes the substitution bug invisible without this test's asymmetric \
             corruption"
        );
        model
            .joint_model_mut(name)
            .unwrap()
            .set_variable_bounds(
                name,
                crate::model::joint::VariableBounds {
                    acceleration_bounded: true,
                    max_acceleration: f64::NAN,
                    // Far from `-max_acceleration`, for the same reason as
                    // the velocity test's `-1e-6`.
                    min_acceleration: -1e-6,
                    ..original
                },
            )
            .unwrap();

        // `panda_joint1` deliberately excluded, so it falls through to the
        // (corrupted) `bounds` instead of an override.
        let mut acceleration_limits_without_joint1 =
            panda_arm_limits([1.3, 2.3, 3.3, 4.3, 5.3, 6.3, 7.3]);
        acceleration_limits_without_joint1.remove(name);

        let mut trajectory = two_waypoint_trajectory(&model);
        let result = compute_time_stamps_with_limits(
            &mut trajectory,
            &HashMap::new(),
            &acceleration_limits_without_joint1,
            &TotgOptions::default(),
        );
        assert!(
            result.is_ok(),
            "corrupted case must still succeed: {result:?}"
        );
        let corrupted_duration = trajectory.duration();

        assert!(
            (corrupted_duration - baseline_duration).abs() < 1e-6,
            "a NaN max_acceleration bound must not be silently replaced by \
             min_acceleration (here, an unrelated 1e-6 rad/s² crawl limit) — the \
             corrupted joint must drop out of the acceleration constraint entirely, \
             leaving duration unchanged from the uncorrupted baseline: baseline \
             {baseline_duration}, corrupted {corrupted_duration}"
        );
    }

    /// `verifyScalingFactor` (cpp:1290-1312): `(0.0, 1.0]` passes through
    /// unchanged, everything else — including the boundary `0.0` itself,
    /// negative, and greater than `1.0` — is replaced by
    /// `DEFAULT_SCALING_FACTOR`.
    #[test]
    fn verify_scaling_factor_boundaries() {
        assert_eq!(verify_scaling_factor(0.5), 0.5);
        assert_eq!(verify_scaling_factor(1.0), 1.0);
        assert_eq!(verify_scaling_factor(0.0), DEFAULT_SCALING_FACTOR);
        assert_eq!(verify_scaling_factor(-0.5), DEFAULT_SCALING_FACTOR);
        assert_eq!(verify_scaling_factor(1.5), DEFAULT_SCALING_FACTOR);
    }

    fn two_waypoint_trajectory(model: &RobotModel) -> RobotTrajectory<'_> {
        let group = model.joint_model_group("panda_arm").unwrap();
        let mut trajectory = RobotTrajectory::for_group(model, Some(group));
        add_panda_arm_waypoint(
            &mut trajectory,
            model,
            [-0.5, -3.52, 1.35, -2.51, -0.88, 0.63, 0.0],
            0.1,
        );
        add_panda_arm_waypoint(
            &mut trajectory,
            model,
            [-0.45, -3.2, 1.2, -2.4, -0.8, 0.6, 0.0],
            0.1,
        );
        trajectory
    }

    /// §172/§153.1: `resample_dt == 0.0` used to saturate
    /// `(duration / 0.0).ceil() as usize` to `usize::MAX` at the point of
    /// consumption, hanging (then exhausting memory) in the
    /// `0..=sample_count` resample loop. Now rejected at construction: a
    /// `TotgOptions` holding `resample_dt = 0.0` cannot exist at all, so
    /// there is nothing downstream left to hang.
    #[test]
    fn resample_dt_zero_is_rejected_not_hung() {
        let result = TotgOptions::default().with_resample_dt(0.0);
        assert!(
            result.is_err(),
            "resample_dt = 0.0 must be rejected: {result:?}"
        );
    }

    /// Same boundary, negative side: `resample_dt < 0.0` used to cast to `0`
    /// (Rust's `as usize` saturates a negative float to zero) at the point
    /// of consumption, producing a silent one-point trajectory with no
    /// error, crash, or log. Now rejected at construction.
    #[test]
    fn resample_dt_negative_is_rejected_not_silently_truncated() {
        let result = TotgOptions::default().with_resample_dt(-0.01);
        assert!(
            result.is_err(),
            "resample_dt < 0.0 must be rejected, not silently truncated: {result:?}"
        );
    }

    /// A `resample_dt` that is positive and finite but small enough that
    /// `duration / resample_dt` approaches `usize::MAX` passes construction
    /// (`with_resample_dt` only checks finite-and-positive) but must still
    /// be rejected downstream: the constructor's `> 0.0` check alone does
    /// not bound the resulting sample count, only [`MAX_RESAMPLE_SAMPLE_COUNT`]
    /// does, and that bound lives in `do_time_parameterization_calculations`
    /// (it depends on `duration`, unknown at construction time) — this is
    /// still an end-to-end test for exactly that reason.
    #[test]
    fn resample_dt_producing_an_unreasonable_sample_count_is_rejected() {
        let model = panda();
        let mut trajectory = two_waypoint_trajectory(&model);
        let limits = panda_arm_limits([1.3, 2.3, 3.3, 4.3, 5.3, 6.3, 7.3]);
        let options = TotgOptions::default().with_resample_dt(1e-300).unwrap();
        let result = compute_time_stamps_with_limits(&mut trajectory, &limits, &limits, &options);
        // `compute_time_stamps_with_limits`/`do_time_parameterization_
        // calculations` reach several `Error::other` sites (missing
        // velocity/acceleration limits, the active-variable-count guard,
        // the sample-count bound); `.is_err()` alone cannot say which
        // fired. This case hits the sample-count bound specifically
        // (confirmed by printing the error before converting this check).
        assert!(
            result
                .as_ref()
                .is_err_and(|e| e.to_string().contains("exceeding the")),
            "an astronomically small resample_dt must be rejected by the sample-count bound: {result:?}"
        );
    }

    /// A subnormal `resample_dt` (below [`f64::MIN_POSITIVE`]) is finite and
    /// positive, so it passes `with_resample_dt`'s `is_finite() && > 0.0`
    /// check, but drives `raw_sample_count` far past
    /// [`MAX_RESAMPLE_SAMPLE_COUNT`] downstream — the same rejection path as
    /// `resample_dt_producing_an_unreasonable_sample_count_is_rejected`,
    /// exercised at the true subnormal boundary rather than merely a small
    /// normal value.
    #[test]
    fn resample_dt_subnormal_is_rejected() {
        let model = panda();
        let mut trajectory = two_waypoint_trajectory(&model);
        let limits = panda_arm_limits([1.3, 2.3, 3.3, 4.3, 5.3, 6.3, 7.3]);
        // f64::from_bits(1), the smallest positive subnormal.
        let subnormal = 5e-324;
        assert!(subnormal > 0.0 && subnormal < f64::MIN_POSITIVE);
        let options = TotgOptions::default().with_resample_dt(subnormal).unwrap();
        let result = compute_time_stamps_with_limits(&mut trajectory, &limits, &limits, &options);
        // See `resample_dt_producing_an_unreasonable_sample_count_is_rejected`
        // for why this checks the message rather than just `.is_err()`.
        assert!(
            result
                .as_ref()
                .is_err_and(|e| e.to_string().contains("exceeding the")),
            "a subnormal resample_dt must be rejected by the sample-count bound: {result:?}"
        );
    }

    /// `resample_dt = NaN` fails `with_resample_dt`'s `is_finite()` check
    /// directly at construction -- distinct from the
    /// `duration == 0.0 && resample_dt == 0.0` NaN documented below, which
    /// arises from division rather than being passed in directly.
    #[test]
    fn resample_dt_nan_is_rejected() {
        let result = TotgOptions::default().with_resample_dt(f64::NAN);
        assert!(
            result.is_err(),
            "resample_dt = NaN must be rejected: {result:?}"
        );
    }

    /// `resample_dt = +inf` fails `with_resample_dt`'s `is_finite()` check
    /// at construction; `duration / +inf == 0.0` would otherwise `.ceil()`
    /// to a harmless-looking `0` downstream, so this must be caught by the
    /// finiteness check specifically, not the sample-count bound.
    #[test]
    fn resample_dt_positive_infinity_is_rejected() {
        let result = TotgOptions::default().with_resample_dt(f64::INFINITY);
        assert!(
            result.is_err(),
            "resample_dt = +inf must be rejected: {result:?}"
        );
    }

    /// `resample_dt = -inf` fails both the finiteness and the `<= 0.0`
    /// check in `with_resample_dt`.
    #[test]
    fn resample_dt_negative_infinity_is_rejected() {
        let result = TotgOptions::default().with_resample_dt(f64::NEG_INFINITY);
        assert!(
            result.is_err(),
            "resample_dt = -inf must be rejected: {result:?}"
        );
    }

    /// A `resample_dt` deliberately sized so `duration / resample_dt` lands
    /// in the immediate vicinity of `usize::MAX` (not just "very large")
    /// passes construction but is still rejected downstream by
    /// [`MAX_RESAMPLE_SAMPLE_COUNT`] before the `as usize` cast, and does
    /// not attempt to loop -- this is checked by asserting rejection, never
    /// by actually driving the `0..=sample_count` loop anywhere near that
    /// count.
    #[test]
    fn resample_dt_targeting_the_usize_max_boundary_is_rejected() {
        let model = panda();
        let mut trajectory = two_waypoint_trajectory(&model);
        let limits = panda_arm_limits([1.3, 2.3, 3.3, 4.3, 5.3, 6.3, 7.3]);
        let approx_duration = 10.0;
        let resample_dt = approx_duration / (usize::MAX as f64);
        assert!(resample_dt.is_finite() && resample_dt > 0.0);
        let options = TotgOptions::default()
            .with_resample_dt(resample_dt)
            .unwrap();
        let result = compute_time_stamps_with_limits(&mut trajectory, &limits, &limits, &options);
        // See `resample_dt_producing_an_unreasonable_sample_count_is_rejected`
        // for why this checks the message rather than just `.is_err()`.
        assert!(
            result
                .as_ref()
                .is_err_and(|e| e.to_string().contains("exceeding the")),
            "a resample_dt targeting the usize::MAX boundary must be rejected by the sample-count bound, not looped: {result:?}"
        );
    }

    /// §172 item 4: `duration == 0.0 && resample_dt == 0.0` would divide to
    /// `NaN`, and `NaN as usize` is `0` in Rust — an accidentally safe
    /// value, not a deliberately validated one. In practice this exact
    /// division is unreachable: two waypoints that collapse to the same
    /// position take the `points.len() == 1` early return above, which
    /// succeeds *without ever reading `resample_dt`* — no two *distinct*
    /// points can produce `duration == 0.0`, since any nonzero path length
    /// takes positive time under finite velocity/acceleration limits. This
    /// is reasoned from the control flow (there is no reachable nonzero-point
    /// zero-duration path to construct), not measured against one.
    ///
    /// Constructs `TotgOptions { resample_dt: 0.0, .. }` via the
    /// `pub(crate)`-visible field directly, bypassing
    /// [`TotgOptions::with_resample_dt`] on purpose: that constructor would
    /// otherwise make this exact scenario impossible to set up at all
    /// (`resample_dt = 0.0` is rejected at construction everywhere outside
    /// this crate). This white-box test exists to pin the *control-flow*
    /// invariant — the early return fires before consumption, independent
    /// of the value — as a second, independent guarantee alongside the
    /// constructor's type-level one.
    ///
    /// This also covers [`totg_compute_time_stamps`]'s internal two-call
    /// chain (see
    /// `totg_compute_time_stamps_silently_collapses_duplicate_waypoints_matching_upstream`
    /// below): its second call passes a `new_resample_dt` computed from the
    /// *first* call's result, which for duplicate input waypoints is
    /// `0.0 / (num_waypoints - 1) == 0.0` (`:586`) — but the trajectory the
    /// first call leaves behind already has exactly one waypoint (this same
    /// `points.len() == 1` branch, hit by the first call), so the second
    /// call's own diversity loop sees `num_points == 1` and takes this same
    /// early return *again*, before ever reading the `0.0` it was handed.
    /// Reproduced directly (`cargo test temp_probe...`, output discarded,
    /// not committed) before writing this: the observed result was
    /// `Ok(())` with `way_point_count == 1`, not an `Err` from the
    /// `resample_dt` validation added elsewhere in this module — confirming
    /// the early return, not a `0.0 / 0.0 = NaN -> 0` cast, is what actually
    /// fires. `moveit2` upstream has the identical two-early-return
    /// structure at `cpp:1219-1226` (read directly, pinned SHA), executed
    /// twice for the same reason, so this is exact behavioural parity, not
    /// a porting deviation — no `§153.1` note applies.
    #[test]
    fn resample_dt_is_unreachable_when_waypoints_collapse_to_one_point() {
        let model = panda();
        let group = model.joint_model_group("panda_arm").unwrap();
        let mut trajectory = RobotTrajectory::for_group(&model, Some(group));
        let position = [-0.5, -3.52, 1.35, -2.51, -0.88, 0.63, 0.0];
        add_panda_arm_waypoint(&mut trajectory, &model, position, 0.1);
        add_panda_arm_waypoint(&mut trajectory, &model, position, 0.1);

        let limits = panda_arm_limits([1.3, 2.3, 3.3, 4.3, 5.3, 6.3, 7.3]);
        let options = TotgOptions {
            resample_dt: 0.0,
            ..Default::default()
        };
        let result = compute_time_stamps_with_limits(&mut trajectory, &limits, &limits, &options);
        assert!(
            result.is_ok(),
            "identical waypoints collapse before resample_dt is ever read: {result:?}"
        );
        assert_eq!(trajectory.way_point_count(), 1);
    }

    /// `totg_compute_time_stamps` asked for `num_waypoints = 5` and two
    /// input waypoints at identical positions; the input has 2 waypoints,
    /// the caller requested (approximately) 5, and what comes back is 1 --
    /// with `Ok(())`, no error, no log. See
    /// `resample_dt_is_unreachable_when_waypoints_collapse_to_one_point`'s
    /// doc comment above for the exact mechanism (the `points.len() == 1`
    /// early return firing twice, once per internal `computeTimeStamps`
    /// call) and why this is verified upstream parity (`cpp:1219-1226`
    /// fires the same way), not a `resample_dt` narrowing bug: the `0.0`
    /// `new_resample_dt` this scenario computes at `:586` is never actually
    /// read as a divisor by either call. This test pins the current,
    /// upstream-matching contract so it is not silently changed later by
    /// someone assuming the missing waypoints are a narrowing-fix
    /// oversight.
    #[test]
    fn totg_compute_time_stamps_silently_collapses_duplicate_waypoints_matching_upstream() {
        let mut model = panda();
        for name in PANDA_ARM_JOINTS {
            let joint = model.joint_model_mut(name).unwrap();
            let mut limits = joint.variable_bounds_msg();
            for limit in &mut limits {
                limit.has_acceleration_limits = true;
                limit.max_acceleration = 3.3;
            }
            joint.set_variable_bounds_from_limits(&limits);
        }
        let group = model.joint_model_group("panda_arm").unwrap();
        let mut trajectory = RobotTrajectory::for_group(&model, Some(group));
        let position = [-0.5, -3.52, 1.35, -2.51, -0.88, 0.63, 0.0];
        add_panda_arm_waypoint(&mut trajectory, &model, position, 0.1);
        add_panda_arm_waypoint(&mut trajectory, &model, position, 0.1);

        let result = totg_compute_time_stamps(5, &mut trajectory, 1.0, 1.0);

        assert!(result.is_ok(), "{result:?}");
        assert_eq!(trajectory.way_point_count(), 1);
        assert_eq!(trajectory.duration(), 0.0);
    }

    /// Q1 (round 4 task), case 1: an empty trajectory is a silent no-op
    /// success (cpp:928-929/1034-1035), not an error.
    #[test]
    fn empty_trajectory_is_a_no_op_success() {
        let model = panda();
        let group = model.joint_model_group("panda_arm").unwrap();
        let mut trajectory = RobotTrajectory::for_group(&model, Some(group));

        let result = compute_time_stamps(&mut trajectory, &TotgOptions::default());
        assert!(result.is_ok());
        assert_eq!(trajectory.way_point_count(), 0);
    }

    /// Q1, case 2: a single-waypoint trajectory collapses to exactly one
    /// waypoint with zero velocity/acceleration and `duration_from_previous
    /// == 0.0` (cpp:1218-1227's `points.size() == 1` branch).
    #[test]
    fn single_waypoint_collapses_to_one_zero_velocity_waypoint() {
        let model = panda();
        let group = model.joint_model_group("panda_arm").unwrap();
        let mut trajectory = RobotTrajectory::for_group(&model, Some(group));
        add_panda_arm_waypoint(
            &mut trajectory,
            &model,
            [-0.5, -3.52, 1.35, -2.51, -0.88, 0.63, 0.0],
            0.1,
        );

        let limits = panda_arm_limits([1.3, 2.3, 3.3, 4.3, 5.3, 6.3, 7.3]);
        compute_time_stamps_with_limits(&mut trajectory, &limits, &limits, &TotgOptions::default())
            .expect("a single waypoint is not an error");

        assert_eq!(trajectory.way_point_count(), 1);
        assert_eq!(trajectory.way_point_duration_from_previous(0), 0.0);
        let waypoint = trajectory.way_point(0).unwrap();
        for name in PANDA_ARM_JOINTS {
            assert_eq!(waypoint.variable_velocity(name).unwrap(), 0.0);
            assert_eq!(waypoint.variable_acceleration(name).unwrap(), 0.0);
        }
    }

    /// Q1, case 3: a trajectory whose waypoints are all identical hits the
    /// exact same `points.size() == 1` collapse as the genuinely-single-
    /// waypoint case above — every later waypoint fails the
    /// `min_angle_change` diversity check against the first
    /// (cpp:1194-1216).
    #[test]
    fn all_identical_waypoints_collapse_the_same_way_as_a_single_waypoint() {
        let model = panda();
        let group = model.joint_model_group("panda_arm").unwrap();
        let positions = [-0.5, -3.52, 1.35, -2.51, -0.88, 0.63, 0.0];

        let mut trajectory = RobotTrajectory::for_group(&model, Some(group));
        add_panda_arm_waypoint(&mut trajectory, &model, positions, 0.1);
        add_panda_arm_waypoint(&mut trajectory, &model, positions, 0.1);
        add_panda_arm_waypoint(&mut trajectory, &model, positions, 0.1);

        let limits = panda_arm_limits([1.3, 2.3, 3.3, 4.3, 5.3, 6.3, 7.3]);
        compute_time_stamps_with_limits(&mut trajectory, &limits, &limits, &TotgOptions::default())
            .expect("all-identical waypoints are not an error");

        assert_eq!(trajectory.way_point_count(), 1);
        assert_eq!(trajectory.way_point_duration_from_previous(0), 0.0);
        let waypoint = trajectory.way_point(0).unwrap();
        for name in PANDA_ARM_JOINTS {
            assert_eq!(waypoint.variable_velocity(name).unwrap(), 0.0);
            assert_eq!(waypoint.variable_acceleration(name).unwrap(), 0.0);
        }
    }

    /// A *consecutive* duplicate in the middle of an otherwise-diverse
    /// trajectory is dropped (cpp:1206-1216's `else if (p == num_points -
    /// 1)` branch never fires for a middle point, so it is simply never
    /// pushed), not collapsed to a single waypoint and not counted twice:
    /// `[A, A, B]` must produce exactly the same total duration as `[A, B]`.
    #[test]
    fn a_middle_duplicate_waypoint_is_dropped_not_double_counted() {
        let model = panda();
        let group = model.joint_model_group("panda_arm").unwrap();
        let a = [-0.5, -3.52, 1.35, -2.51, -0.88, 0.63, 0.0];
        let b = [-0.45, -3.2, 1.2, -2.4, -0.8, 0.6, 0.0];
        let limits = panda_arm_limits([1.3, 2.3, 3.3, 4.3, 5.3, 6.3, 7.3]);

        let mut with_dup = RobotTrajectory::for_group(&model, Some(group));
        add_panda_arm_waypoint(&mut with_dup, &model, a, 0.1);
        add_panda_arm_waypoint(&mut with_dup, &model, a, 0.1);
        add_panda_arm_waypoint(&mut with_dup, &model, b, 0.1);
        compute_time_stamps_with_limits(&mut with_dup, &limits, &limits, &TotgOptions::default())
            .expect("a dropped duplicate is not an error");

        let mut without_dup = RobotTrajectory::for_group(&model, Some(group));
        add_panda_arm_waypoint(&mut without_dup, &model, a, 0.1);
        add_panda_arm_waypoint(&mut without_dup, &model, b, 0.1);
        compute_time_stamps_with_limits(
            &mut without_dup,
            &limits,
            &limits,
            &TotgOptions::default(),
        )
        .unwrap();

        assert!((with_dup.duration() - without_dup.duration()).abs() < 1e-9);
        assert_eq!(with_dup.way_point_count(), without_dup.way_point_count());
    }

    /// Q2: a group with a mimic joint. `r_end_effector`'s active joints
    /// (`r_gripper_l_finger_joint`, `r_gripper_motor_slider_joint`,
    /// `r_gripper_motor_screw_joint`, `r_gripper_joint` — 4 variables) are a
    /// strict subset of its full variable list (7, the same 4 plus the 3
    /// joints that mimic `r_gripper_l_finger_joint`:
    /// `r_gripper_r_finger_joint`, `r_gripper_l_finger_tip_joint`,
    /// `r_gripper_r_finger_tip_joint`). See the module-level "Deviations
    /// from upstream" note: this port rejects the mismatch outright rather
    /// than reproducing upstream's out-of-bounds write.
    #[test]
    fn mimic_joint_group_is_a_typed_error_not_a_panic() {
        let model = pr2();
        let group = model.joint_model_group("r_end_effector").unwrap();
        assert_eq!(group.active_joint_indices().len(), 4);
        assert_eq!(group.variable_names().len(), 7);

        let mut trajectory = RobotTrajectory::for_group(&model, Some(group));
        let mut first = RobotState::new(&model);
        first.set_to_default_values();
        trajectory.add_suffix_way_point(first.clone(), 0.0).unwrap();
        let mut second = first.clone();
        second
            .set_variable_position("r_gripper_l_finger_joint", 0.3)
            .unwrap();
        trajectory.add_suffix_way_point(second, 0.1).unwrap();

        let mut limits = HashMap::new();
        for name in [
            "r_gripper_l_finger_joint",
            "r_gripper_motor_slider_joint",
            "r_gripper_motor_screw_joint",
            "r_gripper_joint",
        ] {
            limits.insert(name.to_string(), 1.0);
        }

        let result = compute_time_stamps_with_limits(
            &mut trajectory,
            &limits,
            &limits,
            &TotgOptions::default(),
        );
        let err = result.expect_err("a mimic-joint group must not silently misparameterize");
        let message = err.to_string();
        assert!(
            message.contains("4") && message.contains('7'),
            "expected the error to name both the active (4) and full (7) variable counts: {message}"
        );
    }

    /// A custom entry in `velocity_limits`/`acceleration_limits` is never
    /// bounds-checked, unlike a `RobotModel`-derived fallback (cpp:1069-1074/
    /// 1103-1108 vs. cpp:1076-1088/1110-1122; see the module-level
    /// "Deviations from upstream" note). A `0.0` custom acceleration limit
    /// must actually reach `Trajectory::create` as `0.0` — not merely avoid
    /// the model-bounds-fallback branch's own "invalid max_acceleration"
    /// message, which `panda()`'s URDF-loaded bounds never set
    /// `acceleration_bounded` for anyway (see `totg_compute_time_stamps_
    /// silently_collapses_duplicate_waypoints_matching_upstream`'s
    /// `joint_model_mut` workaround), so that branch's absence proves
    /// nothing here. Asserting on the `Trajectory::create` failure's own
    /// distinguishing phrase instead (bite-confirmed: neutralizing the
    /// custom-limit-applied flag alone changes the error to "no
    /// acceleration limit was defined" and this assertion still passed
    /// against the old negative check).
    #[test]
    fn a_zero_custom_limit_skips_bound_validation() {
        let model = panda();
        let group = model.joint_model_group("panda_arm").unwrap();
        let mut trajectory = RobotTrajectory::for_group(&model, Some(group));
        add_panda_arm_waypoint(
            &mut trajectory,
            &model,
            [-0.5, -3.52, 1.35, -2.51, -0.88, 0.63, 0.0],
            0.1,
        );
        add_panda_arm_waypoint(
            &mut trajectory,
            &model,
            [-0.45, -3.2, 1.2, -2.4, -0.8, 0.6, 0.0],
            0.1,
        );

        let velocity_limits = panda_arm_limits([1.3, 2.3, 3.3, 4.3, 5.3, 6.3, 7.3]);
        let mut acceleration_limits = panda_arm_limits([1.3, 2.3, 3.3, 4.3, 5.3, 6.3, 7.3]);
        acceleration_limits.insert("panda_joint1".to_string(), 0.0);

        let result = compute_time_stamps_with_limits(
            &mut trajectory,
            &velocity_limits,
            &acceleration_limits,
            &TotgOptions::default(),
        );
        // A zero max_acceleration for a joint that actually moves between the two
        // waypoints (panda_joint1 does) still fails downstream, inside
        // `Trajectory::create` — matching upstream's own
        // `testRelevantZeroMaxAccelerationsInvalidateTrajectory` (already ported
        // in round 3). Assert on `Trajectory::create`'s own distinguishing
        // phrase, not on the absence of the model-bounds-fallback branch's
        // message: that branch is unreachable for this fixture regardless of
        // whether the custom limit was actually applied (see the doc comment
        // above), so its absence cannot prove the custom `0.0` was used.
        const DISTINGUISHING_PHRASE: &str = "after integrateForward and integrateBackward";
        let err = result.expect_err("a zero acceleration limit on a moving joint is invalid");
        let message = err.to_string();
        assert!(
            message.contains(DISTINGUISHING_PHRASE),
            "a custom zero acceleration limit must actually reach Trajectory::create: {message}"
        );
    }

    /// `hasMixedJointTypes` (cpp:1273-1288): `panda_arm` is all-revolute;
    /// `r_end_effector` mixes revolute (`r_gripper_l_finger_joint`,
    /// `r_gripper_motor_screw_joint`) and prismatic
    /// (`r_gripper_motor_slider_joint`, `r_gripper_joint`) active joints.
    #[test]
    fn has_mixed_joint_types_boundary() {
        let panda_model = panda();
        let panda_group = panda_model.joint_model_group("panda_arm").unwrap();
        let panda_trajectory = RobotTrajectory::for_group(&panda_model, Some(panda_group));
        assert!(!has_mixed_joint_types(&panda_trajectory, panda_group));

        let pr2_model = pr2();
        let pr2_group = pr2_model.joint_model_group("r_end_effector").unwrap();
        let pr2_trajectory = RobotTrajectory::for_group(&pr2_model, Some(pr2_group));
        assert!(has_mixed_joint_types(&pr2_trajectory, pr2_group));
    }

    /// A moving joint with a custom `0.0` velocity limit is reachable
    /// through this public group-driven API too, not just direct
    /// `Trajectory::create`/`Path::create` calls (see `trajectory.rs`'s
    /// `a_max_velocity_component_of_zero_is_rejected_rather_than_crawling_to_infinity`
    /// for the same mechanism and the deliberate deviation from upstream
    /// `time_optimal_trajectory_generation.cpp:405` it documents).
    /// `Trajectory::create` now rejects this construction directly, via
    /// `compute_time_stamps_with_limits`'s `Trajectory::create(...)?`, so
    /// this no longer reaches `raw_sample_count.is_finite()`'s downstream
    /// net at all — this test used to be that net's load-bearing case
    /// (bite-confirmed: neutralizing `!is_finite()` alone used to turn this
    /// into a silent `Ok(())` with a NaN-derived `sample_count`, saturating
    /// to `0` under `as usize`); the `!is_finite()` clause's remaining
    /// coverage, if any, is not re-derived here since covering it was never
    /// the point of this deviation. `resample_dt_producing_an_
    /// unreasonable_sample_count_is_rejected`/`resample_dt_targeting_the_
    /// usize_max_boundary_is_rejected` keep the `> MAX_RESAMPLE_SAMPLE_
    /// COUNT` half covered.
    #[test]
    fn resample_dt_over_an_infinite_time_construction_is_rejected() {
        let model = panda();
        let group = model.joint_model_group("panda_arm").unwrap();
        let mut trajectory = RobotTrajectory::for_group(&model, Some(group));
        add_panda_arm_waypoint(
            &mut trajectory,
            &model,
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            0.1,
        );
        add_panda_arm_waypoint(
            &mut trajectory,
            &model,
            [1e-5, 1e-5, 0.0, 0.0, 0.0, 0.0, 0.0],
            0.1,
        );

        let mut velocity_limits = panda_arm_limits([1.0; 7]);
        velocity_limits.insert("panda_joint1".to_string(), 0.0);
        let acceleration_limits = panda_arm_limits([1.0; 7]);

        let result = compute_time_stamps_with_limits(
            &mut trajectory,
            &velocity_limits,
            &acceleration_limits,
            &TotgOptions {
                min_angle_change: 0.0,
                ..TotgOptions::default()
            },
        );
        // See `resample_dt_producing_an_unreasonable_sample_count_is_rejected`
        // for why this checks the message rather than just `.is_err()`.
        assert!(
            result.as_ref().is_err_and(|e| e
                .to_string()
                .contains("bridging the gap would require infinite time")),
            "a zero relative velocity across a nonzero position change must \
             be rejected by Trajectory::create itself: {result:?}"
        );
    }

    /// `totgComputeTimeStamps` (cpp:1137-1160) requires `num_waypoints >
    /// 1` (cpp:1147-1151).
    #[test]
    fn totg_compute_time_stamps_rejects_fewer_than_two_waypoints() {
        let model = panda();
        let group = model.joint_model_group("panda_arm").unwrap();
        let mut trajectory = RobotTrajectory::for_group(&model, Some(group));
        add_panda_arm_waypoint(
            &mut trajectory,
            &model,
            [-0.5, -3.52, 1.35, -2.51, -0.88, 0.63, 0.0],
            0.1,
        );

        // `totg_compute_time_stamps` reaches more than one `Error::other`
        // site (its own num_waypoints guard, plus everything
        // `compute_time_stamps` can fail with); a bare `.is_err()` cannot
        // say which fired (assertion-discrimination-round2.md sec. 3).
        assert!(
            totg_compute_time_stamps(1, &mut trajectory, 1.0, 1.0)
                .unwrap_err()
                .to_string()
                .contains("num_waypoints > 1")
        );
        assert!(
            totg_compute_time_stamps(0, &mut trajectory, 1.0, 1.0)
                .unwrap_err()
                .to_string()
                .contains("num_waypoints > 1")
        );
    }
}
