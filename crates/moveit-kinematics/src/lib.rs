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
//! `chainiksolver_vel_mimic_svd.cpp`/`.hpp` define or override, plus every
//! `KinematicsBase` method they actually exercise. `ported as` names the
//! Rust item; `excluded` cites the `PORTING-PLAN.md` decision.
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
//! Every other `KinematicsBase` virtual is **not ported**, because
//! `KDLKinematicsPlugin` never overrides it and this port has no other
//! caller for it either:
//!
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
//! needed yet." This audit found no gap requiring a new port.

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
