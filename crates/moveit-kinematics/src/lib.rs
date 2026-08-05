// Copyright (c) 2008, Willow Garage, Inc.
// Copyright (c) 2012, Willow Garage, Inc.
// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// Ported from moveit2 @ e017c91ee12984393a28ba246075c65f69cde3bf:
//   moveit_core/kinematics_base/include/moveit/kinematics_base/kinematics_base.hpp
//   moveit_core/kinematics_base/src/kinematics_base.cpp
//   moveit_kinematics/kdl_kinematics_plugin/src/kdl_kinematics_plugin.cpp
//   moveit_kinematics/kdl_kinematics_plugin/include/moveit/kdl_kinematics_plugin/kdl_kinematics_plugin.hpp
//   moveit_kinematics/kdl_kinematics_plugin/include/moveit/kdl_kinematics_plugin/joint_mimic.hpp
// See the module doc's "Why this file stays BSD-3-Clause" section for
// chainiksolver_vel_mimic_svd.{hpp,cpp}, the LGPL-2.1-or-later source this
// crate's velocity solve (`velocity.rs`) plays the role of instead of
// porting.

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
//! # Why this file stays BSD-3-Clause
//!
//! moveit2 vendors `KDL::ChainIkSolverVelMimicSVD`
//! (`chainiksolver_vel_mimic_svd.{hpp,cpp}`) inside its own
//! `moveit_kinematics/kdl_kinematics_plugin/` tree, cited the same way as
//! this crate's legitimately-BSD `kdl_kinematics_plugin.{hpp,cpp}` and
//! `kinematics_base.{hpp,cpp}` citations above — but that one vendored file
//! keeps its own original header: `Copyright (C) 2007 Ruben Smits`,
//! `URL: http://www.orocos.org/kdl`, LGPL-2.1-or-later, modified for mimic
//! joints under `Copyright (C) 2013 Sachin Chitta, Willow Garage` inside
//! that same LGPL file — heavier copyleft than this workspace's
//! BSD-3-Clause. A moveit2 file's citation *path* does not say which
//! license applies; only its own header does.
//!
//! `velocity.rs`'s mimic-fold/velocity-solve plays the role of that file's
//! `CartToJnt` without transcribing it — see that module's own doc comment
//! for the derivation. What this crate's audit below reuses from
//! `chainiksolver_vel_mimic_svd.{hpp,cpp}` is exclusively *interface
//! facts*: which upstream method name a Rust item corresponds to (a
//! pointer for cross-reference, not expression) and the 6-row
//! linear/angular twist convention every caller must already agree on.
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
//! - `CartToJnt(JntArray, Twist, JntArray&, weights)` — plays the role of
//!   `velocity::solve_velocity`, independently derived rather than
//!   transcribed (see that module's own "Why this file stays BSD-3-Clause"
//!   section — its source is LGPL-2.1-or-later).
//! - `jacToJacReduced` — re-derived as `velocity::fold_jacobian`; the
//!   inverse qdot-expansion loop is re-derived as `velocity::expand_to_full`.
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
//! `moveit_kinematics`'s three other plugins.
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
//! - `cached_ik_kinematics_plugin` — **ported**, as
//!   `ik_cache`-backed [`CachedIkSolver`], registered under
//!   `"newton_raphson_cached"`/`"lma_cached"` in [`KINEMATICS_SOLVERS`].
//!   `CachedIKKinematicsPlugin<KinematicsPlugin>` is a CRTP mixin that
//!   wraps *any* `KinematicsBase` solver, intercepting `searchPositionIK`/
//!   `getPositionIK` to replace the caller's seed with the nearest cached
//!   `(pose, solution)` pair (`IKCache::getBestApproximateIKSolution`, a
//!   GNAT nearest-neighbor search over past solutions,
//!   `detail/NearestNeighborsGNAT.hpp`) before delegating to the wrapped
//!   solver, then caches the new pair on success
//!   (`IKCache::updateCache`). Confirmed by reading every method body in
//!   `cached_ik_kinematics_plugin-inl.hpp`, upstream overrides **six**
//!   `KinematicsBase` virtuals with this pattern (`initialize`,
//!   `getPositionIK`, four `searchPositionIK` overloads) — not the one the
//!   name might suggest. That count does not change the port's shape:
//!   [`KinematicsSolver::solve_with_options`] had already collapsed the
//!   five non-`initialize` overloads into one method before this type
//!   existed (see that method's own doc comment), so [`CachedIkSolver`]
//!   wraps exactly that one method; `initialize` has no Rust-side
//!   counterpart to wrap since construction already happens once, in
//!   [`SolverRegistration::construct`]. See [`CachedIkSolver`]'s own doc
//!   comment for this reasoning spelled out in full, plus the deviation
//!   this collapse implies for timeout budgeting.
//!
//!   **What did not port, and why:**
//!
//!   1. *Linear scan, not a GNAT tree.* Upstream's own default
//!      `max_cache_size` is 5000 (`cached_ik_kinematics_parameters.yaml`);
//!      every shipped example config (`config/kdl_cached.yaml`,
//!      `trac_cached.yaml`, `ur5_cached.yaml`) raises it to 10000. A linear
//!      scan over at most 10k entries of the cheap position+quaternion
//!      distance metric (`ik_cache`'s `pose_distance`) is microseconds,
//!      not a bottleneck worth porting `detail/NearestNeighborsGNAT.hpp`
//!      (755 lines implementing a general-purpose metric tree) to avoid.
//!   2. *The on-disk cache format is a local choice, not a port target.*
//!      `ik_cache.cpp`'s `saveCache`/`initializeCache` read and write a
//!      raw, unversioned `memcpy` of `double` fields with no endianness
//!      handling, keyed into the filename by robot/group/frame name plus
//!      the cache size and distance thresholds. Nothing outside this one
//!      C++ class ever reads that file, so this port keeps the *feature*
//!      — a cache that survives the process — and drops the byte layout:
//!      `ik_cache::format` serializes through `serde_json`, the crate this
//!      workspace already pins with `float_roundtrip` so every `f64`
//!      returns bit-for-bit. That module's doc is the single place the
//!      format is defined and the single place to change it; it also lists
//!      which parts of upstream's save *policy* (the write inside
//!      `updateCache` every 500 entries, the write from `~IKCache`, the
//!      filename mangling) are deliberately not ported, and why the
//!      threshold-in-the-filename trick is replaced by carrying the
//!      options inside the document. [`CachedIkSolver::save_cache`] and
//!      [`CachedIkSolver::from_cache_file`] are the caller-facing ends.
//!   3. *`cached_ur_kinematics_plugin.cpp` — out of D1/D2 scope, not
//!      ported.* Read in full: the entire file is a
//!      `PLUGINLIB_EXPORT_CLASS` registration of
//!      `CachedIKKinematicsPlugin<ur_kinematics::URKinematicsPlugin>`.
//!      `URKinematicsPlugin` is an external, UR-specific closed-form
//!      analytic solver from the separate `ur_kinematics` ROS package —
//!      this crate does not have it and is not porting it, for the same
//!      "no portable algorithm exists" reason `ikfast_kinematics_plugin`
//!      is excluded above. With no `URKinematicsPlugin` to wrap, there is
//!      nothing in this file for [`CachedIkSolver`] to instantiate over.
//!      `cached_ik_kinematics_plugin.cpp` (the KDL/Srv/TracIK
//!      registrations) is likewise not ported: it is pluginlib
//!      `PLUGINLIB_EXPORT_CLASS` boilerplate, and this crate's compile-time
//!      [`KINEMATICS_SOLVERS`] registry (decision D4) replaces that
//!      mechanism entirely rather than reproducing it.
//!
//!   Registration needed one `construct` function per wrapped solver, not
//!   one generic entry: [`SolverRegistration::construct`] is a bare `fn`
//!   pointer, not a closure, so a single generic `CachedIkSolver<S>`
//!   cannot itself appear once in [`KINEMATICS_SOLVERS`] parameterized by
//!   which solver it wraps — upstream hit the identical shape problem and
//!   answered it with one `.cpp` file per instantiation
//!   (`cached_kdl_kinematics_plugin.cpp`, the excluded
//!   `cached_ur_kinematics_plugin.cpp`). This port's equivalent is the two
//!   `#[linkme::distributed_slice]` statics at the bottom of
//!   `cached_solver.rs`, one per solver this crate ships
//!   (`newton_raphson`, `lma`).
//!
//!   **`public:` member coverage, by name.** There is no separate
//!   `ik_cache.hpp` — `IKCache`'s declaration (only its `.cpp` is
//!   `ik_cache.cpp`) lives inside `cached_ik_kinematics_plugin.hpp`
//!   alongside `IKCacheMap`, `CachedIKKinematicsPlugin`, and
//!   `CachedMultiTipIKKinematicsPlugin`. That one header has four `public:`
//!   blocks. A prior version of this audit gave "36" without stating how it
//!   counted, and a reviewer re-deriving it by counting `(` on declaration
//!   lines inside each `public:` block got 41, then 27 after subtracting
//!   continuation lines/doc comments/an inline body — neither matched, and
//!   neither count could be checked against the other without knowing what
//!   each one meant by "one member". This version states that first:
//!
//!   - Constructors and destructors **count**, including `~IKCache()`,
//!     even though Rust's ownership model gives them no direct counterpart
//!     — they are still real public declarations a reader of the header
//!     sees.
//!   - `= delete`d declarations (`IKCache(const IKCache&) = delete;`)
//!     **count**, same reason.
//!   - `Options`'s and `Pose`'s public data members (`max_cache_size`,
//!     `position`, ...) **count individually** — the struct itself is not
//!     one line item with its fields folded in.
//!   - A signature that wraps across multiple source lines **counts once**
//!     — one function, however its parameter list is line-wrapped.
//!   - Doc comments and an inline method body (`initialize`'s, the one
//!     `public:` member defined in-header rather than declared) are not
//!     declarations and are not counted.
//!
//!   By that count, the four `public:` blocks hold 44 members total:
//!
//!   | class | members | ported | not ported |
//!   |---|---|---|---|
//!   | `IKCache` | 20 | 9 | 11 |
//!   | `IKCacheMap` | 6 | 0 | 6 |
//!   | `CachedIKKinematicsPlugin` | 12 | 6 | 6 |
//!   | `CachedMultiTipIKKinematicsPlugin` | 6 | 0 | 6 |
//!   | **total** | **44** | **15** | **29** |
//!
//!   `IKCache`'s 20, by name:
//!
//!   - `Options::Options()` — ported, as [`IkCacheOptions`]'s `Default` impl
//!   - `Options::max_cache_size` — ported, same name
//!   - `Options::min_pose_distance` — ported, same name
//!   - `Options::min_joint_config_distance` — ported, renamed
//!     [`IkCacheOptions::min_config_distance`]
//!   - `Options::cached_ik_path` — not ported. It is the *directory* the
//!     mangled cache filename is placed in, and there is no mangled
//!     filename here to place: [`CachedIkSolver::save_cache`] and
//!     [`CachedIkSolver::from_cache_file`] take the whole path (item 2
//!     above)
//!   - `Pose::Pose() = default` — not ported; no public `Pose` type exists
//!     here (position/orientation go through [`moveit_geometry::Isometry3`]
//!     directly)
//!   - `Pose::Pose(const geometry_msgs::msg::Pose&)` — not ported; no ROS
//!     message types are ported at all ("Do not port the ROS surface")
//!   - `Pose::position` — not ported; folded into `Isometry3::translation`,
//!     an existing type, not re-declared here
//!   - `Pose::orientation` — not ported; folded into `Isometry3::rotation`
//!   - `Pose::distance` — ported, as `ik_cache::pose_distance`, a free
//!     function rather than a method since there is no `Pose` type to hang
//!     it on
//!   - `IKCache::IKEntry` (the multi-tip `pair<vector<Pose>, vector<double>>`
//!     alias) — not ported (multi-tip, out of scope)
//!   - `IKCache::IKCache()` — ported, as `IkCache::new`
//!   - `IKCache::~IKCache()` — not ported. Beyond Rust `Drop` needing no
//!     user declaration, upstream's body is a `saveCache()` call whose
//!     failure it cannot report; saving here is
//!     [`CachedIkSolver::save_cache`], which returns a
//!     [`moveit_error::Result`]
//!   - `IKCache::IKCache(const IKCache&) = delete` — not ported; Rust types
//!     are non-`Copy` by default, so there is no equivalent declaration to
//!     make
//!   - `getBestApproximateIKSolution(const Pose&) const` — ported, as
//!     `IkCache::nearest`
//!   - `getBestApproximateIKSolution(const vector<Pose>&) const` — not
//!     ported (multi-tip)
//!   - `initializeCache(...)` — ported, split by what each half does: the
//!     option assignment is `IkCache::new`'s body, the file read is
//!     `IkCache::load` (through `ik_cache::format`'s `from_json`), and the
//!     filename mangling is the one part with no counterpart — see item 2
//!     above
//!   - `updateCache(const IKEntry&, const Pose&, const vector<double>&)
//!     const` — ported, as `IkCache::update`
//!   - `updateCache(const IKEntry&, const vector<Pose>&, const
//!     vector<double>&) const` — not ported (multi-tip)
//!   - `verifyCache(kdl_kinematics_plugin::KDLKinematicsPlugin&) const` —
//!     not ported (debug-only; takes a live `KDLKinematicsPlugin&` to
//!     re-run FK against, and nothing in this port constructs one
//!     standalone to call it against)
//!
//!   `IKCacheMap`'s 6 and `CachedMultiTipIKKinematicsPlugin`'s 6: not
//!   "multi-tip, out of scope" as a class-level label — each of the 12
//!   checked individually against `moveit2/moveit_kinematics/` in full
//!   (`grep -rn IKCacheMap`/`grep -rn CachedMultiTipIKKinematicsPlugin`,
//!   both re-run 2026-08-04, restricted to `*.cpp`/`*.hpp`/`*.h`) for (a)
//!   whether the member's value/effect is a branch condition elsewhere and
//!   (b) whether upstream has a real production caller — the same two
//!   questions that caught `setGroupStateValidityCallback`'s
//!   misclassification elsewhere in this port (a header-only setter that
//!   turned out to gate IK-solution acceptance in its `.cpp`). Neither
//!   question changes the verdict here, but for a different reason each
//!   time:
//!
//!   `IKCacheMap`'s 6 (`IKEntry`/`Pose` aliases, `IKCacheMap()`,
//!   `~IKCacheMap()`, `getBestApproximateIKSolution`, `updateCache`) —
//!   **(b) fails first, so (a) is moot.** `IKCacheMap` ("a container of IK
//!   caches for cases where there is no fixed base frame",
//!   `cached_ik_kinematics_plugin.hpp:160`) is declared and defined
//!   (`ik_cache.cpp:295-345`) but never constructed: the grep's only hits
//!   are the class's own declaration and its own method definitions, in
//!   those same two files. No `.cpp` anywhere under `moveit_kinematics/`
//!   (or the rest of `moveit2/`, checked the same way) ever writes
//!   `IKCacheMap `/`new IKCacheMap`/`IKCacheMap<`. In particular,
//!   `CachedMultiTipIKKinematicsPlugin` -- upstream's only multi-tip
//!   consumer, and so the one place `IKCacheMap` might plausibly be used
//!   -- holds a plain `IKCache` (`CachedIKKinematicsPlugin::cache_`,
//!   inherited) and calls straight into `IKCache`'s own multi-tip
//!   overloads (`getBestApproximateIKSolution(const vector<Pose>&)`,
//!   `updateCache(nearest, vector<Pose>&, config)`,
//!   `cached_ik_kinematics_plugin-inl.hpp:206,219`) -- not `IKCacheMap` at
//!   all. With zero construction sites, none of the 6 members ever
//!   executes, so none can be a branch condition for anything: not a
//!   defect this crate is declining to port, upstream's own class is dead
//!   code.
//!
//!   `CachedMultiTipIKKinematicsPlugin`'s 6 (`Pose`/`IKEntry`/
//!   `IKCallbackFn`/`KinematicsQueryOptions` aliases, `initialize`,
//!   `searchPositionIK`) — **(b) fails the same way, but (a) is worth
//!   stating separately for the two methods**, since their shape (a
//!   `bool` returned from a `KinematicsBase` override) is exactly the
//!   shape that was a gate, not a setter, in the
//!   `setGroupStateValidityCallback` case. `CachedMultiTipIKKinematicsPlugin`
//!   is a template (`cached_ik_kinematics_plugin.hpp:346`, bodies in
//!   `-inl.hpp:73,196`); grepping all of `moveit2/` for the class name
//!   finds only that declaration and those two definitions -- no `.cpp`
//!   anywhere instantiates it as `CachedMultiTipIKKinematicsPlugin<
//!   SomeConcretePlugin>` (contrast `cached_ik_kinematics_plugin.cpp`,
//!   which does instantiate the single-tip `CachedIKKinematicsPlugin<
//!   KDLKinematicsPlugin>`/`<TracIKKinematicsPlugin>`/... — no sibling
//!   file does the analogous thing for the multi-tip class). So yes,
//!   `initialize`'s and `searchPositionIK`'s returned `bool`s *would* gate
//!   a real caller's control flow if this template were ever
//!   instantiated -- but it never is, anywhere in upstream, so that gate
//!   has no reachable caller to matter to. Excluded per
//!   [`crate::registry::KinematicsSolver`]'s own documented deviation 1
//!   (this crate ports exactly the single-tip shape `kdl_kinematics_
//!   plugin` — the solver it actually wraps — exercises), not merely
//!   restated as "multi-tip".
//!
//!   `CachedIKKinematicsPlugin`'s 12, by name:
//!
//!   - `Pose`, `IKEntry`, `IKCallbackFn`, `KinematicsQueryOptions` (4
//!     `using` aliases) — none ported; no standalone Rust type exists for
//!     the first two (see `IKCache` above), and [`SolveOptions`] already
//!     folds what `KinematicsQueryOptions` plus the solution-callback
//!     parameter carry, so there is nothing for the other two to alias
//!   - `CachedIKKinematicsPlugin()` — ported, as `CachedIkSolver::new`
//!   - `~CachedIKKinematicsPlugin()` — not ported; moot, as above
//!   - `initialize(...)` — not ported; no Rust-side counterpart, explained
//!     above. Its private `initCache` tail — which is where upstream reads
//!     an existing cache file — is [`CachedIkSolver::from_cache_file`],
//!     a second constructor rather than a step inside the only one
//!   - `getPositionIK(...)` and all four `searchPositionIK(...)` overloads
//!     (5 total) — ported, folded into
//!     [`KinematicsSolver::solve_with_options`] via [`CachedIkSolver`], the
//!     six-collapses-to-one fold explained above

mod cached_solver;
mod cart_to_jnt;
mod chain;
mod ik_cache;
mod lma;
mod newton_raphson;
mod params;
mod registry;
mod velocity;

pub use cached_solver::CachedIkSolver;
pub use ik_cache::IkCacheOptions;
pub use lma::LevenbergMarquardtSolver;
pub use newton_raphson::NewtonRaphsonSolver;
pub use params::SolverParams;
pub use registry::{
    DEFAULT_SOLVER_NAME, KINEMATICS_SOLVERS, KinematicsSolver, SolutionCallback, SolveOptions,
    SolverRegistration, resolve_solver,
};
