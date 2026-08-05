// Copyright (c) 2008, Willow Garage, Inc.
// Copyright (c) 2011-2012, Georgia Tech Research Corporation
// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// Ported from moveit2 @ e017c91ee12984393a28ba246075c65f69cde3bf:
//   moveit_core/trajectory_processing/include/moveit/trajectory_processing/time_optimal_trajectory_generation.hpp (lines 62-192)
//   moveit_core/trajectory_processing/src/time_optimal_trajectory_generation.cpp
//   moveit_core/robot_trajectory/include/moveit/robot_trajectory/robot_trajectory.hpp
//   moveit_core/robot_trajectory/src/robot_trajectory.cpp

//! Two related but independent pieces of upstream trajectory handling:
//!
//! - The model-independent numeric core of time-optimal trajectory
//!   generation (Kunz & Stilman, "Time-Optimal Trajectory Generation for
//!   Path Following with Bounded Acceleration and Velocity"): [`Path`] (with
//!   its private [`PathSegment`](path_segment) kinds) and [`Trajectory`],
//!   the parts of upstream's `time_optimal_trajectory_generation.hpp`/`.cpp`
//!   that operate purely on `Vec<DVector<f64>>` waypoints and per-joint
//!   velocity/acceleration bounds, with no
//!   `moveit_core::robot_model`/`robot_trajectory` dependency anywhere.
//! - [`robot_trajectory::RobotTrajectory`], upstream's `robot_trajectory::
//!   RobotTrajectory` — a sequence of `RobotState` waypoints plus
//!   per-waypoint durations. Unlike the two types above, this one *does*
//!   depend on `moveit-model`/`moveit-state`; see that module's doc comment.
//! - [`ruckig_smoothing`], upstream's `trajectory_processing::
//!   RuckigSmoothing` — re-parameterizes a [`robot_trajectory::RobotTrajectory`]
//!   so it also satisfies jerk limits, via the `ruckig` online trajectory
//!   generator (the `rsruckig` crate). See that module's doc comment for
//!   what it does not port.
//! - [`time_optimal_trajectory_generation`], upstream's `trajectory_processing::
//!   TimeOptimalTrajectoryGeneration` — the `robot_trajectory::RobotTrajectory`
//!   adapter around [`Path`]/[`Trajectory`] (header line 193 on). See that
//!   module's doc comment for what it does not port.
//! - [`trajectory_tools`], upstream's `trajectory_processing::trajectory_tools`
//!   free functions — the convenience entry points that wrap
//!   [`time_optimal_trajectory_generation`]/[`ruckig_smoothing`]. See that
//!   module's doc comment for which of the five are ported.
//!
//! # Symbol audit: every public symbol under `trajectory_processing/include/`
//!
//! Re-run by re-reading the headers fresh, not by inferring from what is
//! already ported. `ruckig_traj_smoothing.h`, `time_optimal_trajectory_generation.h`,
//! `time_parameterization.h` and `trajectory_tools.h` are all deprecated
//! auto-generated forwarding shims to the `.hpp` of the same stem (`#pragma
//! message(".h header is obsolete...")`, then one `#include`); no independent
//! content, so only the four `.hpp` files carry real symbols. `ported as
//! <symbol>` gives the Rust name; `D-decision excludes it` names the
//! decision; `unported` gives the reason it is not (yet, or ever) ported.
//!
//! `MOVEIT_CLASS_FORWARD(TimeParameterization)`/
//! `MOVEIT_CLASS_FORWARD(TimeOptimalTrajectoryGeneration)` — both unported:
//! the macro expands to six typedefs via `MOVEIT_DECLARE_PTR` — `*Ptr`/
//! `*ConstPtr` (`std::shared_ptr`), `*WeakPtr`/`*ConstWeakPtr`
//! (`std::weak_ptr`), `*UniquePtr`/`*ConstUniquePtr` (`std::unique_ptr`):
//! this port has no ownership handle to name for either type.
//!
//! ## `time_parameterization.hpp`
//!
//! - `TimeParameterization` (abstract base, 3 pure-virtual `computeTimeStamps`
//!   overloads + defaulted special members) — not ported; see this crate's
//!   own "Not ported: `TimeParameterization`" doc section on
//!   [`time_optimal_trajectory_generation`] for the full reasoning (one
//!   implementor, zero polymorphic call sites anywhere upstream, and the
//!   third overload is D1-blocked regardless).
//!
//! ## `time_optimal_trajectory_generation.hpp`
//!
//! - `DEFAULT_PATH_TOLERANCE` (constant) — ported as [`DEFAULT_PATH_TOLERANCE`].
//! - `LimitType` (enum), `LIMIT_TYPES` (map) — D-decision excludes both: their
//!   only upstream use is formatting the `velocity_`/`acceleration_` prefix
//!   in `verifyScalingFactor`'s `RCLCPP_WARN` message
//!   (`time_optimal_trajectory_generation.cpp:1290-1310`), and this crate
//!   carries no `RCLCPP_WARN` calls at all (see
//!   [`time_optimal_trajectory_generation`]'s "Out of scope" note) —
//!   [`time_optimal_trajectory_generation`]'s private `verify_scaling_factor`
//!   is ported without a `LimitType` parameter because it has nothing left
//!   to do with one.
//! - `PathSegment` (abstract base: `getLength`/`position_`/`getConfig`/
//!   `getTangent`/`getCurvature`/`getSwitchingPoints`/`clone`, pure virtual
//!   except `getLength`) — collapsed into the closed sum type
//!   `path_segment::PathSegment` (a `Linear`/`Circular` [`Clone`]-derived
//!   enum, not virtual dispatch; `pub(crate)`, hence plain code text rather
//!   than a doc link here); see that module's own doc comment for why.
//!   `getLength`/`position_`/`getConfig`/`getTangent`/`getCurvature`/
//!   `getSwitchingPoints` ported as `PathSegment::length`/`position`/
//!   `config`/`tangent`/`curvature`/`switching_points`, all `pub(crate)` —
//!   see the module doc's note that nothing in it is reachable from outside
//!   the crate upstream either. `clone` (virtual) is subsumed by the derived
//!   [`Clone`] impl. `LinearPathSegment`/`CircularPathSegment` are not
//!   declared in this header at all (`.cpp`-only,
//!   `time_optimal_trajectory_generation.cpp`) — ported as
//!   `path_segment::circular::Circular`/`path_segment::linear::Linear` (both
//!   private modules, undocumented in this audit's header-symbol scope
//!   since they carry no header declaration to audit against).
//! - `Path` (class) — ported as [`Path`]:
//!   - `create` (static, `std::optional<Path>`) — ported as [`Path::create`]
//!     (`Result<Self>`: this crate's standing policy, set by this session's
//!     original task brief, is that upstream returning `std::optional` means
//!     failure is a value, and stays a value here rather than becoming a
//!     panic).
//!   - Copy constructor — subsumed by `#[derive(Clone)]` on [`Path`] (a
//!     consequence of `path_segment::PathSegment` being a plain enum instead
//!     of a `Vec<Box<dyn Trait>>`, which needs no hand-written deep copy).
//!   - `getLength`/`getConfig`/`getTangent`/`getCurvature` — ported as
//!     [`Path::length`]/[`Path::config`]/[`Path::tangent`]/[`Path::curvature`].
//!   - `getNextSwitchingPoint`/`getSwitchingPoints` — ported as
//!     `Path::next_switching_point`/`Path::switching_points`, both
//!     `pub(crate)` rather than `pub`: `Trajectory::next_acceleration_switching_point`
//!     is the only caller anywhere in this crate, same as upstream (only
//!     `Trajectory::getNextAccelerationSwitchingPoint` calls the `Path`
//!     equivalent, at `time_optimal_trajectory_generation.cpp:476`), so
//!     nothing needs the wider surface — narrower than upstream's fully
//!     public method, not a gap.
//!   - Default constructor (private upstream, "use `create` instead") — no
//!     Rust equivalent exists either; [`Path::create`] is [`Path`]'s only
//!     constructor.
//!   - `getPathSegment` (private) — ported as the private
//!     `Path::segment_at`-equivalent lookup inlined at each of the three
//!     public methods' call sites in `path.rs` (`config`/`tangent`/
//!     `curvature`, matching upstream's `getConfig`/`getTangent`/
//!     `getCurvature`, `time_optimal_trajectory_generation.cpp:321,327,333`),
//!     rather than kept as one separate function; behaviourally identical,
//!     no upstream call site outside those same three methods to preserve a
//!     shared name for.
//! - `Trajectory` (class) — ported as [`Trajectory`]:
//!   - `create` (static, `std::optional<Trajectory>`) — ported as
//!     [`Trajectory::create`] (`Result<Self>`, same optional-to-Result
//!     mapping as [`Path::create`]).
//!   - `getDuration`/`getPosition`/`getVelocity`/`getAcceleration` — ported
//!     as [`Trajectory::duration`]/[`Trajectory::position`]/
//!     [`Trajectory::velocity`]/[`Trajectory::acceleration`].
//!   - Private constructor, `TrajectoryStep` (nested struct) — ported as
//!     `trajectory.rs`'s own private `TrajectoryStep`.
//!   - `getNextSwitchingPoint`/`getNextAccelerationSwitchingPoint`/
//!     `getNextVelocitySwitchingPoint`/`integrateForward`/`integrateBackward`/
//!     `getMinMaxPathAcceleration`/`getMinMaxPhaseSlope`/
//!     `getAccelerationMaxPathVelocity`/`getVelocityMaxPathVelocity`/
//!     `getAccelerationMaxPathVelocityDeriv`/`getVelocityMaxPathVelocityDeriv`
//!     — ported as `trajectory.rs`'s private `next_switching_point`/
//!     `next_acceleration_switching_point`/`next_velocity_switching_point`/
//!     `integrate_forward`/`integrate_backward`/`min_max_path_acceleration`/
//!     `min_max_phase_slope`/`acceleration_max_path_velocity`/
//!     `velocity_max_path_velocity`/`acceleration_max_path_velocity_deriv`/
//!     `velocity_max_path_velocity_deriv` — the numerics transcribed as-is,
//!     not rewritten: this session's original task brief called for
//!     transcribing the delicate switching-point-search numerics rather than
//!     rewriting them into something cleaner, and that standing policy is
//!     applied here.
//!   - `getTrajectorySegment` (with its `cached_time_`/
//!     `cached_trajectory_segment_` forward-scan cache) — ported as the
//!     private `Trajectory::segment_index`, *without* the cache: see
//!     `segment_index`'s own doc comment for why dropping it is behaviour
//!     preserving (`partition_point` over a list already sorted by
//!     construction finds the identical index the cache's linear scan would
//!     have).
//! - `TimeOptimalTrajectoryGeneration` (class) — the constructor and three
//!   `computeTimeStamps` overloads ported as free functions in
//!   [`time_optimal_trajectory_generation`]; see that module's own "Not
//!   ported: `TimeParameterization`" and "Out of scope" doc sections for
//!   the third (`moveit_msgs::msg::JointLimits`) overload's D1 exclusion,
//!   and "Deviations from upstream" for every behavioural transcription
//!   choice below:
//!   - Constructor (`path_tolerance`/`resample_dt`/`min_angle_change`,
//!     defaulted) — ported as [`time_optimal_trajectory_generation::TotgOptions`].
//!   - `computeTimeStamps` (scaling-only) — ported as
//!     [`time_optimal_trajectory_generation::compute_time_stamps`].
//!   - `computeTimeStamps` (`velocity_limits`/`acceleration_limits` maps) —
//!     ported as
//!     [`time_optimal_trajectory_generation::compute_time_stamps_with_limits`].
//!   - `computeTimeStamps` (`std::vector<moveit_msgs::msg::JointLimits>`) —
//!     D-decision excludes it: D1, thin wrapper around the overload above.
//!   - `doTimeParameterizationCalculations` (private) — ported as
//!     [`time_optimal_trajectory_generation`]'s private
//!     `do_time_parameterization_calculations`.
//!   - `hasMixedJointTypes` (private) — ported as the `pub` standalone
//!     [`time_optimal_trajectory_generation::has_mixed_joint_types`]; wider
//!     visibility than upstream's private method, deliberately: see that
//!     module's "Deviations from upstream" note on why (not called from
//!     `do_time_parameterization_calculations` here, same as upstream only
//!     ever using it for a dropped `RCLCPP_WARN`, but exposed for a caller
//!     — or a test — that wants the diagnostic directly).
//!   - `verifyScalingFactor` (private) — ported as
//!     [`time_optimal_trajectory_generation`]'s private `verify_scaling_factor`
//!     (no `LimitType` parameter; see that enum's entry above).
//! - `totgComputeTimeStamps` (free function, num-waypoints resampling) —
//!   ported as
//!   [`time_optimal_trajectory_generation::totg_compute_time_stamps`]; see
//!   that function's own doc comment for the one upstream call-ordering bug
//!   (a discarded first-call failure) this port does not reproduce.
//!
//! ## `trajectory_tools.hpp`
//!
//! See [`trajectory_tools`]'s own module doc for the full citation of each;
//! summarized here for audit completeness:
//!
//! - `isTrajectoryEmpty` — D-decision excludes it: D1 (`moveit_msgs::msg::
//!   RobotTrajectory` parameter).
//! - `trajectoryWaypointCount` — D-decision excludes it: D1, same reason.
//! - `applyTOTGTimeParameterization` — ported as
//!   [`trajectory_tools::apply_totg_time_parameterization`].
//! - `applyRuckigSmoothing` — ported as
//!   [`trajectory_tools::apply_ruckig_smoothing`].
//! - `createTrajectoryMessage` — D-decision excludes it: D1 (`trajectory_msgs::
//!   msg::JointTrajectory` return type; no ROS type in its parameters, but
//!   D1 excludes a signature for appearing on either side, not just as
//!   input).
//!
//! ## `ruckig_traj_smoothing.hpp`
//!
//! See [`ruckig_smoothing`]'s own module doc for the full citation of each;
//! summarized here for audit completeness:
//!
//! - `applySmoothing` (scaling-only) — ported as
//!   [`ruckig_smoothing::apply_smoothing`].
//! - `applySmoothing` (`velocity_limits`/`acceleration_limits`/`jerk_limits`
//!   maps) — ported as [`ruckig_smoothing::apply_smoothing_with_limits`].
//! - `applySmoothing` (`std::vector<moveit_msgs::msg::JointLimits>`) —
//!   D-decision excludes it: D1, thin wrapper around the overload above.
//! - `validateGroup` (private) — ported as [`ruckig_smoothing`]'s private
//!   `validate_group`.
//! - `getRobotModelBounds` (private) — ported as [`ruckig_smoothing`]'s
//!   private `set_robot_model_bounds`; see that module's "Deviations from
//!   upstream" note on why it is infallible here where upstream declares it
//!   `[[nodiscard]] bool`.
//! - `getNextRuckigInput` (private) — ported as [`ruckig_smoothing`]'s
//!   private `get_next_ruckig_input`.
//! - `initializeRuckigState` (private) — ported as [`ruckig_smoothing`]'s
//!   private `initialize_ruckig_state`.
//! - `runRuckig` (private) — ported as [`ruckig_smoothing`]'s private
//!   `run_ruckig`.
//! - `extendTrajectoryDuration` (private) — ported as [`ruckig_smoothing`]'s
//!   private `extend_trajectory_duration`; see that module's own "Deviations
//!   from upstream" note on the header/`.cpp` doc-comment mismatch this port
//!   resolved by following the `.cpp` definition.
//! - `checkOvershoot` (private) — ported as [`ruckig_smoothing`]'s private
//!   `check_overshoot`.
//!
//! # Completion condition
//!
//! This section is a check, not a claim: it names exactly what "done" means
//! for this crate's current scope, so plan and code can be compared directly
//! instead of re-diverging silently (the pattern `moveit-distance-field`'s
//! own "Completion condition" section established, after PORTING-PLAN.md
//! §65/§71 caught a plan claim nobody could verify against the code).
//!
//! **Headers, fully audited (read in full against the pinned SHA, not
//! inferred from what is already ported):**
//!
//! - `moveit_core/trajectory_processing/include/moveit/trajectory_processing/{time_parameterization,time_optimal_trajectory_generation,trajectory_tools,ruckig_traj_smoothing}.hpp`
//!   plus their four `.h` deprecated-forwarding-shim siblings (no independent
//!   content) — see the "Symbol audit: every public symbol under
//!   `trajectory_processing/include/`" section above for the per-symbol
//!   table.
//! - `moveit_core/robot_trajectory/include/moveit/robot_trajectory/robot_trajectory.hpp`
//!   plus its `.h` shim — see the "Symbol audit: `robot_trajectory.hpp`"
//!   section below for its per-symbol table.
//!
//! Every symbol in both headers is classified in those two sections as
//! ported (with its Rust name), D-decision-excluded (with the decision), or
//! unported (with the specific reason) — there is no symbol from either
//! header left unclassified.
//!
//! **Fixtures, and what they cover:**
//!
//! - `tests/totg_parity.rs` — the oracle's `totg` op, core-only branch (no
//!   top-level `"group"` key): [`Path`]/[`Trajectory`] end to end, one case
//!   per invariant boundary (below the two-waypoint minimum, duplicate
//!   consecutive waypoints, a zero-length path, a velocity-saturating
//!   straight line, `upstream_test2`'s general case).
//! - `tests/totg_path_parity.rs` — the oracle's `totg_path` op:
//!   [`Path`] geometry alone (`length`/`config`/`tangent`/`curvature`), no
//!   [`Trajectory`] timing, against `upstream_test2`'s five waypoints
//!   sampled strictly inside each blend segment. Isolates `Path`
//!   construction from `Trajectory::create`'s switching-point search, so a
//!   future regression in one does not hide inside the other's tolerance.
//! - `tests/totg_robot_trajectory_parity.rs` — the same `totg` op's
//!   group-driven branch (`compute_time_stamps_with_limits`, the
//!   `RobotTrajectory` adapter) against a real `panda_arm` trajectory.
//! - `tests/totg_robot_trajectory_scaling_only_parity.rs` — the scaling-only
//!   overload ([`time_optimal_trajectory_generation::compute_time_stamps`]),
//!   closing the gap [`time_optimal_trajectory_generation`]'s own "Closed
//!   gap" doc section describes.
//! - `tests/totg_synthetic_parity.rs` — the `totg_synthetic` model fixture:
//!   a multi-DOF `planar` joint group and a mixed prismatic/revolute group,
//!   covering `active_joint_variables`'s per-joint expansion and
//!   [`time_optimal_trajectory_generation::has_mixed_joint_types`] against
//!   more than the trivial single-variable case.
//! - `tests/large_accel.rs` — upstream `testLargeAccel`
//!   (`test_time_optimal_trajectory_generation.cpp`), against
//!   upstream's own fixture data extracted verbatim into
//!   `tests/fixtures/large_accel_waypoints.json` (see that file's own module
//!   doc for why JSON rather than retyped literals).
//! - [`trajectory`]'s and [`path`](path_segment)'s own `#[cfg(test)]`
//!   modules — every case `test_time_optimal_trajectory_generation.cpp`'s
//!   gtest suite carries for [`Path`]/[`Trajectory`]
//!   (`upstream_test1`/`upstream_test2`/`upstream_test3`/
//!   `upstream_test_single_dof_discontinuity`/`upstream_test_custom_limits`
//!   and friends), plus invariant-boundary cases the suite does not carry
//!   (two waypoints, three collinear waypoints, duplicate consecutive
//!   waypoints, `max_deviation` of `0.0`, a limit vector containing a zero,
//!   zero total path length).
//! - `tests/ruckig_parity.rs` — the oracle's `ruckig` op against
//!   [`ruckig_smoothing::apply_smoothing`]/
//!   [`ruckig_smoothing::apply_smoothing_with_limits`].
//! - `tests/ruckig_smoothing.rs` — every upstream `RuckigTests` case
//!   (`test_ruckig_traj_smoothing.cpp`), plus boundary tests for
//!   `apply_smoothing`/`apply_smoothing_with_limits`'s own invariants
//!   (missing group, empty/single-waypoint trajectories, duplicate
//!   waypoints).
//! - `tests/robot_trajectory.rs` — every `test_robot_trajectory.cpp` case
//!   except the five named in that file's own header comment
//!   (`RobotTrajectoryShallowCopy`, needing `shared_ptr` waypoint aliasing
//!   this port deliberately does not have; `ChainEdits`/`DoubleReverse`,
//!   adapted to drop the `*RobotTrajectoryMsg` steps, D1;
//!   `MultiDofTrajectoryToJointStates`/`SetMultiDofTrajectory`, D1), plus
//!   boundary tests for the `duration_from_previous[0] == 0.0` invariant and
//!   typed-error index access.
//! - [`robot_trajectory`]'s and [`time_optimal_trajectory_generation`]'s own
//!   `#[cfg(test)]` modules, and `tests/trajectory_tools.rs`'s wrapper
//!   `#[cfg(test)]` module — unit/boundary tests for behaviour with no
//!   oracle op to compare against: typed-error index access, mimic-joint
//!   dimension-mismatch rejection, `has_mixed_joint_types` on a group with
//!   no mixed types, and (`trajectory_tools.rs`) that each convenience
//!   wrapper forwards its arguments to the function it wraps identically to
//!   calling that function directly.
//!
//! Every oracle-backed fixture above is registered in
//! `tests/fixtures/oracle-models.json` (`ruckig`, `totg`, `totg_path`,
//! `totg_robot_trajectory`, `totg_robot_trajectory_scaling_only`,
//! `totg_synthetic`, each naming the URDF/SRDF pair its request/response
//! JSON was captured against), and every key there matches a real
//! `op == "..."` (`totg_path` is its own dispatch case, distinct from the
//! `"group"`-key branches inside `op == "totg"` that the three other
//! `totg_*` variants use) in `tools/moveit-oracle/src/oracle.cpp`.
//!
//! **What is still missing, and why it is not a gap in the above:** every
//! item is already named individually in the two symbol-audit sections
//! (above and below) with its own reason; this is the roll-up.
//! `TimeParameterization` is unported because it has exactly one upstream
//! implementor and zero polymorphic call sites — see
//! [`time_optimal_trajectory_generation`]'s own "Not ported:
//! `TimeParameterization`" doc section. Every `moveit_msgs`/
//! `trajectory_msgs`-typed overload or return value (the third
//! `computeTimeStamps` overload, `trajectory_tools`'s `isTrajectoryEmpty`/
//! `trajectoryWaypointCount`/`createTrajectoryMessage`, `ruckig_traj_smoothing`'s
//! third `applySmoothing` overload, and `robot_trajectory`'s
//! `getRobotTrajectoryMsg`/three `setRobotTrajectoryMsg` overloads/
//! `toJointTrajectory`) is D1-excluded and belongs in the optional
//! `moveit-ros` crate instead, not this one. `LimitType`/`LIMIT_TYPES` and
//! the hand-rolled `RobotTrajectory::Iterator` are D-decision-excluded for
//! reasons specific to each (an `RCLCPP_WARN`-only use this crate has no
//! logging channel for; an idiomatic-iterator replacement respectively) —
//! see their own symbol-audit entries. `RobotTrajectory::print`/
//! `operator<<` is ported as `impl std::fmt::Display for RobotTrajectory`
//! (round 12); see `robot_trajectory.rs`'s "Deviations from upstream" note
//! for the one gap that trait signature leaves (no `variable_indexes`
//! override parameter).
//!
//! This crate's completion condition, stated as a check rather than a
//! claim: every symbol in both audited headers is classified above; every
//! classified-as-ported symbol has either an upstream-gtest-derived fixture,
//! an oracle-driven fixture, or a boundary/unit test with a documented
//! reason no oracle op covers it; and every classified-as-unported symbol
//! names the specific missing dependency or D-decision. If a future symbol
//! or fixture cannot be placed in one of those buckets, this section is
//! stale and needs re-auditing before the plan is updated to match it.
//!
//! # Symbol audit: `robot_trajectory.hpp`
//!
//! Re-run by re-reading the header fresh, not by inferring from what is
//! already ported. `robot_trajectory.h` is the deprecated forwarding shim
//! (`#pragma message(".h header is obsolete...")`, one `#include`) — no
//! independent content.
//!
//! `MOVEIT_CLASS_FORWARD(RobotTrajectory)` — unported: expands to six
//! typedefs via `MOVEIT_DECLARE_PTR` — `RobotTrajectoryPtr`/`ConstPtr`
//! (`std::shared_ptr`), `WeakPtr`/`ConstWeakPtr` (`std::weak_ptr`),
//! `UniquePtr`/`ConstUniquePtr` (`std::unique_ptr`); this port has no
//! ownership handle to name.
//!
//! - 3 constructors (`RobotModelConstPtr`; `+ group: const std::string&`;
//!   `+ group: const JointModelGroup*`) — ported as
//!   [`robot_trajectory::RobotTrajectory::new`]/
//!   [`robot_trajectory::RobotTrajectory::for_group_name`]/
//!   [`robot_trajectory::RobotTrajectory::for_group`]. See that module's
//!   "Deviations from upstream" note: an unknown group name is `Err`, not a
//!   silent whole-robot fallback (upstream's `getJointModelGroup` logs and
//!   returns `nullptr`).
//! - `operator=` (defaulted, shallow copy) and the `(other, deepcopy)` copy
//!   constructor — distinct: subsumed by `#[derive(Clone)]`, which always
//!   deep-copies (see that module's "Deviations from upstream" note on why
//!   there is no cheaper aliasing mode left to preserve — `RobotState` is
//!   already a plain value type here, not a `shared_ptr`).
//! - `getRobotModel`/`getGroup`/`getGroupName`/`setGroupName` — ported as
//!   `robot_trajectory::RobotTrajectory::robot_model`/`group`/`group_name`/
//!   `set_group_name`. `setGroupName` upstream silently sets `group_` to
//!   `nullptr` for an unknown name; this port's `set_group_name` returns
//!   `Err` instead, the same deviation as the string-group constructor.
//! - `getWayPointCount`/`size` — both ported as one
//!   `robot_trajectory::RobotTrajectory::way_point_count`. Upstream's `size`
//!   differs from `getWayPointCount` only by a debug-mode `assert` that
//!   `waypoints_`/`duration_from_previous_` stay the same length; this
//!   port's two `VecDeque`s can never diverge in length in the first place
//!   (every mutator keeps them in lockstep), so there is nothing left for a
//!   second method to assert.
//! - `getWayPoint`/`getLastWayPoint`/`getFirstWayPoint` (`const&`, UB out of
//!   range) and `getWayPointPtr`/`getLastWayPointPtr`/`getFirstWayPointPtr`
//!   (mutable) — ported as `way_point`/`last_way_point`/`first_way_point`
//!   and their `_mut` counterparts, all `Result`-returning; see that
//!   module's "Deviations from upstream" note on panicking index access
//!   becoming `Result`. Upstream's `const`/mutable overload pairs collapse
//!   to one name each plus a `_mut` suffix, the idiomatic Rust shape.
//! - `getWayPointDurations` — ported as `way_point_durations`.
//! - `getWayPointDurationFromStart` — ported as
//!   `way_point_duration_from_start`; clamps an out-of-range `index` to the
//!   last waypoint, matching upstream's own doc comment ("returns overall
//!   duration if index is out of range").
//! - `getWayPointDurationFromPrevious` — ported as
//!   `way_point_duration_from_previous`; `0.0` out of range, matching
//!   upstream.
//! - `setWayPointDurationFromPrevious` — ported as
//!   `set_way_point_duration_from_previous`; `Err` at index 0 for a nonzero
//!   value, per the `duration_from_previous[0]` invariant (that module's
//!   doc comment).
//! - `empty` — ported as `is_empty`.
//! - `addSuffixWayPoint` (2 overloads: `const RobotState&`, `const
//!   RobotStatePtr&`) — collapsed into one
//!   `add_suffix_way_point(RobotState<'m>, dt)`: this port's `RobotState` is
//!   already a plain value type, with no `shared_ptr`/`Ptr` distinction to
//!   preserve two overloads for. Calls [`moveit_state::RobotState::update`]
//!   on the incoming state before storing it, matching upstream's
//!   `state->update()` — see that method's own doc comment for why this
//!   matters beyond upstream-parity (`RobotState`'s derived `PartialEq`).
//! - `addPrefixWayPoint` (2 overloads) — collapsed the same way into
//!   `add_prefix_way_point(RobotState<'m>)`; upstream's `dt` parameter is
//!   dropped (that module's doc: the front waypoint's duration is
//!   structurally `0.0`). Also calls `update()` before storing.
//! - `insertWayPoint` (2 overloads) — collapsed into
//!   `insert_way_point(index, RobotState<'m>, dt)`, `Result`-returning where
//!   upstream indexes unchecked. Also calls `update()` before storing.
//! - `append` — ported as `append`; upstream's `start_index = 0`/
//!   `end_index = SIZE_MAX` defaults are dropped (Rust has no default
//!   parameters) — every caller passes both explicitly. Does not call
//!   `update()`: unlike the three methods above, `append` only moves
//!   already-stored (already-settled) waypoints from `source`, matching
//!   upstream, which does not call `update()` here either.
//! - `swap` — distinct: `std::mem::swap(&mut a, &mut b)` already swaps two
//!   [`robot_trajectory::RobotTrajectory`] values whole, at zero cost and
//!   with identical behaviour to a bespoke method — this port's struct has
//!   no `Drop` impl and no `shared_ptr`-style aliasing to preserve
//!   field-by-field (upstream's own `swap` exists specifically to call
//!   `robot_model_.swap(...)` on a `shared_ptr` rather than incur an
//!   atomic-refcount copy/decrement pair). No forwarding wrapper is added.
//! - `removeWayPoint` — ported as `remove_way_point`, `Result`-returning.
//! - `clear` — ported as `clear`.
//! - `getDuration` — ported as `duration`.
//! - `getAverageSegmentDuration` — ported as `average_segment_duration`.
//! - `getRobotTrajectoryMsg` — D-decision excludes it: D1
//!   (`moveit_msgs::msg::RobotTrajectory` output parameter).
//! - `setRobotTrajectoryMsg(reference_state, const
//!   trajectory_msgs::msg::JointTrajectory&)` — D-decision excludes it: D1.
//! - `setRobotTrajectoryMsg(reference_state, const
//!   moveit_msgs::msg::RobotTrajectory&)` — D-decision excludes it: D1.
//! - `setRobotTrajectoryMsg(reference_state, const moveit_msgs::msg::
//!   RobotState&, const moveit_msgs::msg::RobotTrajectory&)` — D-decision
//!   excludes it: D1 (two `moveit_msgs` parameters).
//! - `reverse` — ported as `reverse`.
//! - `unwind()`/`unwind(const RobotState&)` — ported as `unwind`/
//!   `unwind_from`.
//! - `findWayPointIndicesForDurationAfterStart` — ported as
//!   `find_way_point_indices_for_duration_after_start`; returns
//!   `(usize, usize, f64)` instead of writing through three out-parameters —
//!   the idiomatic Rust shape for a 3-value return, not a behavioural
//!   deviation. All 4 documented edge cases (empty trajectory, negative
//!   duration, duration past the total, single-waypoint trajectory) are
//!   reproduced and covered by their own boundary tests.
//! - `getStateAtDurationFromStart` — ported as
//!   `state_at_duration_from_start`; see that module's "Deviations from
//!   upstream" note on the `Option` return replacing the out-parameter/
//!   `bool` pair.
//! - `class Iterator`, `begin`/`end` — excluded, see that module's "Out of
//!   scope" note: `iter()` is the idiomatic replacement.
//! - `print`/`operator<<` — ported as `impl std::fmt::Display for
//!   RobotTrajectory`; see that module's "Deviations from upstream" note.
//! - free `pathLength` — ported as `path_length`.
//! - free `smoothness` — ported as `smoothness`.
//! - free `waypointDensity` — ported as `waypoint_density`.
//! - free `toJointTrajectory` — D-decision excludes it: D1
//!   (`trajectory_msgs::msg::JointTrajectory` return type; no ROS type in
//!   its parameters, but D1 excludes a signature for appearing on either
//!   side).

mod numeric;
mod path;
pub mod path_segment;
pub mod robot_trajectory;
pub mod ruckig_smoothing;
pub mod time_optimal_trajectory_generation;
pub mod trajectory;
pub mod trajectory_tools;

pub use path::{DEFAULT_PATH_TOLERANCE, Path};
pub use robot_trajectory::RobotTrajectory;
pub use trajectory::Trajectory;
