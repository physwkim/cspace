// Copyright (c) 2008, Willow Garage, Inc.
// Copyright (c) 2012, Willow Garage, Inc.
// Copyright (c) 2007, Ruben Smits
// Copyright (c) 2013, Sachin Chitta, Willow Garage
// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// Ported from moveit2 @ e017c91ee12984393a28ba246075c65f69cde3bf:
//   moveit_core/kinematics_base/include/moveit/kinematics_base/kinematics_base.hpp
//   moveit_core/kinematics_base/src/kinematics_base.cpp
//   moveit_kinematics/kdl_kinematics_plugin/src/kdl_kinematics_plugin.cpp
//   moveit_kinematics/kdl_kinematics_plugin/include/moveit/kdl_kinematics_plugin/kdl_kinematics_plugin.hpp
//   moveit_kinematics/kdl_kinematics_plugin/src/chainiksolver_vel_mimic_svd.cpp
//   moveit_kinematics/kdl_kinematics_plugin/include/moveit/kdl_kinematics_plugin/chainiksolver_vel_mimic_svd.hpp
//   moveit_kinematics/kdl_kinematics_plugin/include/moveit/kdl_kinematics_plugin/joint_mimic.hpp

//! Numeric inverse kinematics for moveit-rs.
//!
//! Upstream splits this across `kinematics_base` (the plugin interface,
//! loaded at runtime through pluginlib by class name) and
//! `kdl_kinematics_plugin` (the one solver every MoveIt robot actually gets
//! by default: a damped/truncated-least-squares Newton iteration over a KDL
//! chain's Jacobian). Per `PORTING-PLAN.md` decision D4, the runtime
//! plugin-by-string-name lookup is not ported — [`KinematicsSolver`]
//! implementations register themselves at compile time in
//! [`KINEMATICS_SOLVERS`] through [`linkme`], and a caller picks one by
//! constructing it directly ([`NewtonRaphsonSolver::new`],
//! [`LevenbergMarquardtSolver::new`]) or by scanning the registry for a
//! [`SolverRegistration::name`].
//!
//! # What upstream reaches through KDL, and what this port reaches through
//! `moveit-state`
//!
//! `kdl_kinematics_plugin` builds a `KDL::Chain` from the URDF and gets its
//! Jacobian and forward kinematics from KDL's own solvers
//! (`ChainJntToJacSolver`, `ChainFkSolverPos_recursive`). This port has no
//! KDL dependency (D1/D2): both come from [`moveit_state::Posed`] —
//! [`moveit_state::Posed::jacobian`] and
//! [`moveit_state::Posed::global_link_transform`] — which already
//! encapsulate the chain-validity checks
//! (`moveit_model::JointModelGroup::is_chain`) this crate would otherwise
//! have to re-derive.
//!
//! # Do not port the ROS surface
//!
//! No `rclcpp::Node`, no `geometry_msgs::Pose`, no
//! `moveit_msgs::MoveItErrorCodes`, no pluginlib registration macro. Poses
//! are [`moveit_geometry::Isometry3`]; errors that mean "this solver cannot
//! be built for this group" go through [`moveit_error::Error`]; failure to
//! converge is not an error (upstream itself models it as a `bool`, not an
//! exception) — see [`KinematicsSolver::solve`].
//! `initialize(node, robot_model, group_name, base_frame, tip_frames,
//! search_discretization)` becomes a plain constructor taking the model, the
//! group name and a [`SolverParams`].
//!
//! # Position-only IK and joint limits are modes, not solvers
//!
//! Matching upstream (`KinematicsQueryOptions`/`params_.position_only_ik`
//! select behaviour inside the one `CartToJnt`, they do not select a
//! different class), [`SolverParams::position_only`] and joint-limit
//! clipping (`chain::ChainInfo`'s bounds, applied every iteration by
//! `cart_to_jnt`'s `clip_to_joint_limits`) are parameters and always-on
//! behaviour of both solvers below, not separate types.
//!
//! # Symbol coverage audit
//!
//! Every method `kdl_kinematics_plugin.cpp`/`.hpp` and
//! `chainiksolver_vel_mimic_svd.cpp`/`.hpp` define or override, plus **every**
//! public `KinematicsBase` symbol in `kinematics_base.hpp` — not only the
//! ones `KDLKinematicsPlugin` happens to exercise; round 9 closed that
//! narrower boundary after finding it left `setValues` unclassified (see the
//! `KinematicsBase` list below). `ported as` names the Rust item; `excluded`
//! cites the `PORTING-PLAN.md` decision; `not ported` names the concretely
//! absent caller.
//!
//! `kdl_kinematics_plugin.{hpp,cpp}`:
//!
//! - `KDLKinematicsPlugin()` (ctor) — trivial member-init only, nothing to
//!   port.
//! - `getPositionIK` — folded into [`KinematicsSolver::solve`]: upstream
//!   itself is a `searchPositionIK` call with `timeout=0.0`, i.e.
//!   `max_restarts=0`.
//! - `searchPositionIK` (3 thin-wrapper overloads: no options, consistency
//!   limits only, full options minus solution callback) — folded into
//!   [`KinematicsSolver::solve`]/[`KinematicsSolver::solve_with_options`]
//!   via [`SolveOptions`]'s defaults; no separate Rust item, since none of
//!   these overloads carry logic beyond constructing the options the
//!   fullest overload takes directly.
//! - `searchPositionIK` (fullest overload: seed, timeout, consistency
//!   limits, options, solution callback, error code) — ported as
//!   `cart_to_jnt::search_position_ik`.
//! - `CartToJnt` (protected, the Newton iteration) — ported as
//!   `cart_to_jnt::cart_to_jnt`.
//! - `getPositionFK` — **not in this crate**. `moveit_state::Posed`
//!   (`global_link_transform`/`global_link_transform_at`) already provides
//!   forward kinematics for any link by name; this crate would only
//!   duplicate that call. Confirmed no `getPositionFK`/`position_fk`
//!   symbol exists anywhere under `crates/moveit-kinematics/src/`.
//! - `initialize` — split three ways:
//!   - The chain/mimic/joint-limit setup (KDL tree/chain build,
//!     `dimension_`, `mimic_joints_`, `joint_min_`/`joint_max_`) is ported
//!     as `chain::ChainInfo::build`.
//!   - The solver-construction tail (resolved joint weights, RNG) is
//!     ported as `NewtonRaphsonSolver::new`/`new_with_seed` and
//!     `LevenbergMarquardtSolver::new`/`new_with_seed`.
//!   - The ROS-parameter loading (`param_listener_`, `params_`,
//!     `storeValues`, `removeSlash`) is **not ported** — it is the ROS
//!     surface this module doc's "Do not port the ROS surface" section
//!     already excludes; `SolverParams` is constructed directly by the
//!     caller instead of read from ROS parameters.
//! - `getJointNames` — ported as `ChainInfo::solver_joint_names`, exposed
//!   through [`KinematicsSolver::joint_names`].
//! - `getLinkNames` — **not ported**. Upstream's only caller is
//!   `getPositionFK`, which is itself not in this crate (see above);
//!   `moveit_state::Posed`'s forward-kinematics calls take a link name
//!   directly rather than querying the solver for the chain's link list,
//!   so there is no consumer left for this method to serve.
//! - `getJointWeights` — ported as `ChainInfo::resolve_joint_weights`.
//! - `timedOut` — **excluded by §4.9** (no wall-clock timeout). Already
//!   cited at the type that replaces it: see
//!   [`SolverParams::max_restarts`]'s doc comment, which names `timedOut`
//!   and `searchPositionIK`'s `do {...} while(!timedOut(...))` loop
//!   explicitly.
//! - `checkConsistency` — ported as `cart_to_jnt::satisfies_consistency`.
//! - `getRandomConfiguration(JntArray&)` — ported as
//!   `cart_to_jnt::random_configuration`.
//! - `getRandomConfiguration(seed, limits, JntArray&)` — ported as
//!   `cart_to_jnt::near_by_configuration`.
//! - `clipToJointLimits` — ported as `cart_to_jnt::clip_to_joint_limits`.
//!
//! `chainiksolver_vel_mimic_svd.{hpp,cpp}`:
//!
//! - `countMimicJoints` (free helper) — folded into
//!   `ChainInfo::build`'s mimic-detection loop; no separate counting pass,
//!   since this port builds `ChainInfo`'s mimic table in the same walk
//!   that would otherwise count it.
//! - `ChainIkSolverVelMimicSVD(...)` (ctor: SVD dimensions, threshold) —
//!   not ported as a persistent object. `velocity::solve_velocity`
//!   constructs `nalgebra::SVD` fresh each call; there is no
//!   long-lived solver state to initialize once.
//! - `updateInternalDataStructures` — upstream's own override is an empty
//!   stub (a `// TODO` comment, no logic); nothing to port.
//! - `CartToJnt(JntArray, Twist, JntArray&)` (delegates to the weighted
//!   overload with unit weights) — folded into `velocity::solve_velocity`,
//!   which takes weights as a parameter rather than defaulting them
//!   through a second entry point.
//! - `CartToJnt(FrameVel, JntArrayVel&)` — upstream itself returns `-1`
//!   ("not yet implemented"); nothing to port.
//! - `CartToJnt(JntArray, Twist, JntArray&, weights)` — ported as
//!   `velocity::solve_velocity`.
//! - `jacToJacReduced` — ported as `velocity::fold_jacobian`; the inverse
//!   qdot-expansion loop is ported as `velocity::expand_to_full`.
//! - `isPositionOnly` — ported inline as
//!   `params.orientation_weight() == 0.0` at `cart_to_jnt`'s call site,
//!   not a standalone method, since [`SolverParams::position_only`]
//!   already carries the same bit.
//!
//! `KinematicsBase` (the interface `KDLKinematicsPlugin` actually
//! overrides — see the two lists above for those methods' Rust homes).
//! Every other public `KinematicsBase` symbol is **not ported**, because
//! `KDLKinematicsPlugin` never overrides or calls it and this port has no
//! other caller for it either:
//!
//! - `setValues` — the alternative to `initialize` for "non-chain IK
//!   solvers" (upstream's own doc comment): a plain field-assignment setter
//!   (`robot_description_`/`group_name_`/`base_frame_`/`tip_frames_`, then
//!   `setSearchDiscretization`) with a base-class body, for a plugin that
//!   builds its model from `robot_description_` directly instead of a
//!   pre-parsed `RobotModel`. `KDLKinematicsPlugin` is chain-based and is
//!   never constructed through it — confirmed by grep, zero references to
//!   `setValues` anywhere under `kdl_kinematics_plugin.{hpp,cpp}` — so
//!   `initialize` (ported, see above) is this crate's only construction
//!   path and `setValues` has no consumer.
//! - `getGroupName`/`getBaseFrame`/`getTipFrame` — the surviving,
//!   single-tip shape of these is [`KinematicsSolver::group_name`],
//!   [`KinematicsSolver::base_frame`], [`KinematicsSolver::tip_frame`].
//! - `getTipFrames` (plural, multi-tip) — excluded per
//!   [`KinematicsSolver`]'s documented single-tip-only deviation.
//! - multi-pose `getPositionIK`/`searchPositionIK`, and the
//!   cost-function `searchPositionIK` overload — `KDLKinematicsPlugin`
//!   never overrides these defaults (it is always called with exactly one
//!   pose); excluded per the same single-tip/no-cost-function deviation.
//! - `setRedundantJoints`/`getRedundantJoints`/`supportsGroup` — unused by
//!   `KDLKinematicsPlugin`, which never calls into redundant-joint
//!   handling; no consumer in this port.
//! - `setSearchDiscretization`/`getSearchDiscretization`/
//!   `getSupportedDiscretizationMethods` — these back a discretized
//!   redundant-joint search `KDLKinematicsPlugin` does not implement; no
//!   consumer in this port.
//! - `setDefaultTimeout`/`getDefaultTimeout` — back the same wall-clock
//!   timeout `timedOut` implements; excluded by §4.9 alongside it.
//! - `storeValues`/`removeSlash` (protected helpers) — used only by
//!   `initialize`'s ROS-parameter loading; excluded alongside it, above.
//!
//! Conclusion: every method with a live consumer in this crate's scope
//! has a Rust-side home; every exclusion above cites a specific
//! `PORTING-PLAN.md` decision or a concretely absent caller, not "not
//! needed yet." This audit found no gap requiring a new port in
//! `kdl_kinematics_plugin` itself — see the next section for
//! `moveit_kinematics`'s three other plugins, one of which *is* a gap.
//!
//! # The other three `moveit_kinematics` plugins
//!
//! `moveit_kinematics/` ships four plugins; the audit above covers only
//! `kdl_kinematics_plugin`, the one every MoveIt robot gets by default.
//! The other three, checked rather than assumed:
//!
//! - `srv_kinematics_plugin` — **excluded, D1/D2 (no ROS dependency)**.
//!   Read `srv_kinematics_plugin.cpp`: its `searchPositionIK` body is an
//!   `rclcpp::Client<moveit_msgs::srv::GetPositionIK>` (`ik_service_client_`,
//!   constructed in `initialize` via `node_->create_client`,
//!   `async_send_request` in `searchPositionIK`) that forwards the request
//!   to an external ROS node and returns its answer. There is no numeric
//!   solver here to port — the entire class *is* the ROS surface this
//!   crate's "Do not port the ROS surface" section already excludes, not a
//!   solver with a ROS wrapper around it.
//! - `ikfast_kinematics_plugin` — **unported, no portable algorithm
//!   exists**. This directory has no `src/`: `templates/
//!   ikfast61_moveit_plugin_template.cpp` is a 1421-line C++ template with
//!   placeholder tokens that OpenRave's separate, external IKFast code
//!   generator fills in with a *robot-specific* closed-form analytic
//!   solution, per the README ("Generates a IKFast kinematics plugin for
//!   MoveIt using OpenRave generated cpp files"). There is no generic
//!   IKFast solver class to port; porting this would mean porting
//!   OpenRave's symbolic-algebra codegen tool, which is a different and
//!   far larger project, not a gap in this crate's scope.
//! - `cached_ik_kinematics_plugin` — **unported, and unlike the two above
//!   this is a real gap, not a decision.** Read
//!   `cached_ik_kinematics_plugin.hpp`/`ik_cache.cpp` rather than assume
//!   from the name: `CachedIKKinematicsPlugin<KinematicsPlugin>` is a CRTP
//!   mixin that inherits from *any* `KinematicsBase` solver (in upstream,
//!   `cached_kdl_kinematics_plugin.cpp`/`cached_ur_kinematics_plugin.cpp`
//!   instantiate it over `KDLKinematicsPlugin`), intercepting
//!   `searchPositionIK` to replace the caller's seed with the nearest
//!   cached `(pose, solution)` pair (`IKCache::getBestApproximateIKSolution`,
//!   a GNAT nearest-neighbor search over past solutions,
//!   `detail/NearestNeighborsGNAT.hpp`) before delegating to the wrapped
//!   solver, then caches the new pair on success (`IKCache::updateCache`,
//!   which both accepts-if-different-enough and persists — there is no
//!   separate confirm step). Cache persistence (`ik_cache.cpp`) is plain
//!   `std::filesystem` file I/O, not a ROS resource lookup. This is
//!   algorithmic, not ROS-bound, and `PORTING-PLAN.md` §4.4 already lists
//!   `cached_ik` alongside `kdl`, `srv`, `ikfast` as one of the four
//!   `KinematicsSolver` trait implementors the plugin-registry decision (D4)
//!   anticipated. No task this crate has received has asked for a
//!   warm-started/cached solver variant, so nothing here builds one — but
//!   the honest reason is "unimplemented, in scope," not "not requested,"
//!   since those read identically in six months and only one is a decision
//!   that closes the gap.
//!
//!   **Scope, if a future task asks for it** (not implemented this round —
//!   this is the estimate the decision needs, not the code):
//!
//!   1. *One trait method, not the crate's `KinematicsBase` surface.* This
//!      crate's [`KinematicsSolver`] already collapsed upstream's five
//!      `searchPositionIK`/`getPositionIK` overloads into
//!      [`KinematicsSolver::solve_with_options`] (`# Deviations`, item 1
//!      on that trait). A wrapper only needs to intercept that one method —
//!      substitute the cache's nearest-seed lookup for the caller's `seed`
//!      argument, delegate to the wrapped solver, insert `(target,
//!      solution)` into the cache on `Some`. `group_name`/`joint_names`/
//!      `base_frame`/`tip_frame` delegate unchanged; there is no
//!      `initialize` to wrap since [`KinematicsSolver`] has no such method
//!      (construction already happens once, in
//!      [`SolverRegistration::construct`]).
//!   2. *Linear scan is honest at these sizes; a GNAT tree is not
//!      proportionate.* Upstream's own default `max_cache_size` is 5000
//!      (`cached_ik_kinematics_parameters.yaml`); every shipped example
//!      config (`config/kdl_cached.yaml`, `trac_cached.yaml`,
//!      `ur5_cached.yaml`) raises it to 10000. A linear scan over at most
//!      10k entries of a cheap position+quaternion distance metric is
//!      microseconds, not a bottleneck this wrapper exists to avoid.
//!      `detail/NearestNeighborsGNAT.hpp` is 755 lines implementing a
//!      general-purpose metric tree — porting it to shave a scan that is
//!      already fast at the sizes MoveIt itself configures would be a
//!      second, disproportionately larger project bolted onto this one.
//!   3. *On-disk format is a local choice, not a port target.* `ik_cache.cpp`'s
//!      `saveCache`/`initializeCache` read and write a raw, unversioned
//!      `memcpy` of `tf2Scalar`/`double` fields with no endianness handling,
//!      keyed into the filename by robot/group/frame name plus the cache
//!      size and distance thresholds. Nothing outside this one C++ class
//!      ever reads that file. A Rust port has no compatibility obligation
//!      to that byte layout and should pick its own serialization (e.g.
//!      `serde`), same as every other local-choice deviation this crate
//!      already documents.
//!   4. *Registration needs one function per wrapped solver, not one
//!      generic entry.* [`SolverRegistration::construct`] is a bare `fn`
//!      pointer (`fn(&RobotModel, &str, &SolverParams) ->
//!      Result<Box<dyn KinematicsSolver>>`), not a closure, so a single
//!      generic `CachedIkSolver<S>` cannot itself appear once in
//!      [`KINEMATICS_SOLVERS`] parameterized by which solver it wraps —
//!      upstream hit the identical shape problem and answered it with one
//!      `.cpp` file per instantiation (`cached_kdl_kinematics_plugin.cpp`,
//!      `cached_ur_kinematics_plugin.cpp`). The Rust equivalent is one
//!      `construct` function per solver this crate ships that should get a
//!      cached variant (today: `newton_raphson`, `lma` — two functions, not
//!      N).
//!
//!   Put together: an `IKCache` (pose-distance metric, `Vec<(Pose,
//!   Vec<f64>)>`, linear-scan nearest, min-distance insert gate) plus a
//!   `CachedIkSolver` wrapper implementing [`KinematicsSolver`] by
//!   delegation is on the order of this crate's smaller existing modules —
//!   no ROS, no GNAT port required. That makes it a next-round-sized task,
//!   not a workspace decision; the workspace-level question, if any, is
//!   only whether a cached variant of `newton_raphson`/`lma` is something a
//!   consumer actually wants.

mod cart_to_jnt;
mod chain;
mod lma;
mod newton_raphson;
mod params;
mod registry;
mod velocity;

pub use lma::LevenbergMarquardtSolver;
pub use newton_raphson::NewtonRaphsonSolver;
pub use params::SolverParams;
pub use registry::{
    KINEMATICS_SOLVERS, KinematicsSolver, SolutionCallback, SolveOptions, SolverRegistration,
};
