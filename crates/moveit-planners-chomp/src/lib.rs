// Copyright (c) 2009, 2012, Willow Garage, Inc.
// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// Ported from moveit2 @ e017c91ee12984393a28ba246075c65f69cde3bf:
//   moveit_planners/chomp/chomp_motion_planner/include/chomp_motion_planner/chomp_parameters.hpp
//   moveit_planners/chomp/chomp_motion_planner/src/chomp_parameters.cpp
//   moveit_planners/chomp/chomp_motion_planner/include/chomp_motion_planner/chomp_utils.hpp
//   moveit_planners/chomp/chomp_motion_planner/include/chomp_motion_planner/chomp_trajectory.hpp
//   moveit_planners/chomp/chomp_motion_planner/src/chomp_trajectory.cpp
//   moveit_planners/chomp/chomp_motion_planner/include/chomp_motion_planner/chomp_cost.hpp
//   moveit_planners/chomp/chomp_motion_planner/src/chomp_cost.cpp
//   moveit_planners/chomp/chomp_motion_planner/include/chomp_motion_planner/chomp_optimizer.hpp
//   moveit_planners/chomp/chomp_motion_planner/src/chomp_optimizer.cpp
//   moveit_planners/chomp/chomp_motion_planner/include/chomp_motion_planner/multivariate_gaussian.hpp

//! CHOMP (Covariant Hamiltonian Optimization for Motion Planning), ported
//! from upstream's `moveit_planners/chomp` package.
//!
//! # `chomp_interface/` is not ported
//!
//! Upstream `moveit_planners/chomp` splits into two subpackages:
//! `chomp_motion_planner/` (the algorithm and its data structures — no ROS
//! type anywhere) and `chomp_interface/` (a `pluginlib` `PlanningContext`
//! adapter exposing it to `move_group`). Only `chomp_motion_planner/` is
//! ported here. `chomp_interface/` is excluded per `PORTING-PLAN.md` D1/D2:
//! this port's core crates reference no ROS type at all, and
//! `chomp_interface`'s reason to exist — being a `pluginlib`-loadable
//! `planning_interface::PlanningContext` — is exactly the `move_group`
//! drop-in shape D1 puts out of scope for the core. Nothing in
//! `chomp_interface/` is algorithmic; it only adapts `ChompPlanner` (see
//! below) to ROS parameters and a live `PlanningScene`.
//!
//! # Scope so far: 7 of 8 upstream files
//!
//! `chomp_motion_planner/` has 8 header/source (or header-only) units.
//! Round 15 ported and audited 3: `chomp_parameters`, `chomp_utils`,
//! `chomp_trajectory`. Round 16 added a 4th: `chomp_cost`. Round 17 added a
//! 5th, `chomp_optimizer`, but only its model/collision-independent numeric
//! core. Round 19 ported the remaining `chomp_optimizer` symbols — the
//! `ChompOptimizer` struct itself, `optimize()`, and its collision-coupled
//! methods — once it was established that its only collision-backend
//! dependency, `getCollisionGradients`, was already ported; see
//! [`optimizer`]'s own module doc for the full account of what changed and
//! why round 17's "collision-coupled, not portable" classification did not
//! hold up. `multivariate_gaussian.hpp` (round 18's 6th file) is not this
//! crate's own copy: it depends on `moveit-sampling::MultivariateGaussian`
//! — see below. `chomp_planner` remains **not yet audited**, deliberately
//! deferred (round 17's brief, Item 3): porting it before the optimizer it
//! drives was solid risked redoing it.
//!
//! # `multivariate_gaussian.hpp`: depends on `moveit-sampling`, not a local copy
//!
//! Round 18 gave this crate its own transcription of
//! `chomp::MultivariateGaussian`, reasoning that upstream's algorithmically
//! identical STOMP sibling class lived in `ros-industrial/stomp`
//! (Apache-2.0) while CHOMP's upstream is BSD-3-Clause — and that porting
//! both under one `SPDX-License-Identifier` in a shared `moveit-sampling`
//! crate would mislabel one side. **That premise was wrong.** The actual
//! upstream path for STOMP's copy that `moveit-sampling` ports from is
//! `moveit_planners/stomp/include/stomp_moveit/math/multivariate_gaussian.hpp`
//! — a file inside the `moveit2` tree itself (confirmed by reading its
//! header: `Software License Agreement (BSD License)`, `Copyright (c) 2009,
//! Willow Garage, Inc.`), not a file from the separate `ros-industrial/stomp`
//! Apache-2.0 repository. Both upstream headers `moveit-sampling` ports from
//! are therefore BSD-3-Clause end to end, and a shared struct under
//! `SPDX-License-Identifier: BSD-3-Clause` mislabels nothing. Round 19
//! deleted this crate's local copy and switched to `moveit-sampling`'s
//! [`MultivariateGaussian`] (re-exported here), after proving byte-for-byte
//! that `MultivariateGaussian::sample_with_covariance` consumes the RNG in
//! the same order/count and produces the same output as the deleted local
//! `sample` — same standard-normal draw loop, same
//! `mean + covariance_cholesky * draws` combination, for the same seeded
//! `ChaCha8Rng`.
//!
//! It has no live consumer yet in this crate beyond `ChompOptimizer`'s
//! `initialize()`, which constructs one per joint
//! (`MultivariateGaussian(Eigen::VectorXd::Zero(...),
//! joint_costs_[i].getQuadraticCostInverse())`) but never samples it: that
//! construction exclusively feeds the Hamiltonian-Monte-Carlo perturbation
//! path (`perturbTrajectory`/`getRandomMomentum`/`updateMomentum`/
//! `updatePositionFromMomentum`), and every call site of that path in
//! `optimize()` is commented out upstream — see [`optimizer`]'s module doc
//! for the full account, including that three of those four methods have no
//! implementation anywhere in `chomp_optimizer.cpp` at all.
//!
//! # Symbol audit: `chomp_parameters.{hpp,cpp}`
//!
//! - `ChompParameters` (class) — ported as [`parameters::ChompParameters`].
//!   All 20 public data members ported as public fields with upstream's
//!   exact default-constructor values; `setRecoveryParams` ported as
//!   [`parameters::ChompParameters::set_recovery_params`];
//!   `setTrajectoryInitializationMethod` ported as
//!   [`parameters::ChompParameters::set_trajectory_initialization_method`],
//!   kept a validated `String` rather than redesigned into an enum — see
//!   that method's doc comment for why. `VALID_INITIALIZATION_METHODS`
//!   ported as [`parameters::VALID_INITIALIZATION_METHODS`]. The default
//!   destructor (`virtual ~ChompParameters()`) has no Rust equivalent to
//!   port; `virtual` exists upstream only so a subclass (none exists in
//!   this package) could override it.
//!
//! # Symbol audit: `chomp_utils.hpp`
//!
//! - `DIFF_RULE_LENGTH` — ported as [`utils::DIFF_RULE_LENGTH`].
//! - `DIFF_RULES` — ported as [`utils::DIFF_RULES`].
//! - `normalizeAnglePositive` — ported as [`utils::normalize_angle_positive`].
//! - `normalizeAngle` — ported as [`utils::normalize_angle`].
//! - `shortestAngularDistance` — ported as [`utils::shortest_angular_distance`].
//! - `robotStateToArray` — ported as [`utils::robot_state_to_array`].
//!
//! # Symbol audit: `chomp_trajectory.{hpp,cpp}`
//!
//! - `ChompTrajectory` (class) — ported as [`trajectory::ChompTrajectory`].
//!   The duration-based and num-points-based constructors are ported as
//!   [`trajectory::ChompTrajectory::from_duration`] and
//!   [`trajectory::ChompTrajectory::from_num_points`]; the copy-with-padding
//!   constructor as [`trajectory::ChompTrajectory::from_source_trajectory`].
//!   The `trajectory_msgs::msg::JointTrajectory`-typed constructor is
//!   excluded (D1): its signature carries a ROS message type directly, and
//!   nothing else in this round's scope constructs a `ChompTrajectory` from
//!   one. `operator()` (both overloads) is ported as
//!   `impl `[`std::ops::Index`]`/`[`std::ops::IndexMut`]` for
//!   `[`trajectory::ChompTrajectory`]` on `(usize, usize)``. All other
//!   accessors and the three `fillIn*` methods, `fillInFromTrajectory`,
//!   `assignCHOMPTrajectoryPointFromRobotState` and `getJointVelocities` are
//!   ported as their `snake_case` equivalents on
//!   [`trajectory::ChompTrajectory`] — see that module's own doc comment for
//!   the full name mapping and every deviation from upstream.
//!   `getFreeTrajectoryBlock`/`getFreeJointTrajectoryBlock` are ported
//!   (round 17, once `chomp_optimizer`'s real call sites were known) as
//!   [`trajectory::ChompTrajectory::free_trajectory_block_mut`]/
//!   [`trajectory::ChompTrajectory::free_joint_trajectory_block_mut`] — see
//!   that module doc's own deviation note. The private `init` has no
//!   separate Rust equivalent: every public constructor allocates its
//!   matrix directly via `DMatrix::zeros` instead.
//!
//! # Symbol audit: `chomp_cost.{hpp,cpp}`
//!
//! - `ChompCost` (class) — ported as [`cost::ChompCost`]. The constructor is
//!   ported as [`cost::ChompCost::new`], with the unused `joint_number`
//!   parameter dropped — see that method's and the module's own doc
//!   comments for why. `getQuadraticCostInverse`/`getQuadraticCost`/`scale`
//!   are ported as [`cost::ChompCost::quadratic_cost_inverse`]/
//!   [`cost::ChompCost::quadratic_cost`]/[`cost::ChompCost::scale`] with no
//!   behavioral change. `getCost`/`getDerivative`/`getMaxQuadCostInvValue`
//!   are ported as [`cost::ChompCost::cost`]/[`cost::ChompCost::derivative`]/
//!   [`cost::ChompCost::max_quad_cost_inv_value`], each now fallible where
//!   upstream would assert, silently return `NaN`, or read out of bounds —
//!   see [`cost`]'s own module doc for the full account, including the
//!   `quad_cost_inv_` decomposition-family check the round-16 dispatch
//!   asked for. The private `getDiffMatrix` is ported as
//!   [`cost::ChompCost`]'s private `diff_matrix`, transcribed tap-for-tap
//!   including its boundary-truncation behavior (not reflected or
//!   renormalized at the ends) — pinned by a dedicated boundary-vs-interior
//!   unit test per the round-16 dispatch's warning that a wrong boundary
//!   rule leaves interior rows looking correct. The default destructor
//!   (`virtual ~ChompCost()`) has no Rust equivalent, same reasoning as
//!   `ChompParameters`'s.
//!
//! # Symbol audit: `chomp_optimizer.{hpp,cpp}`
//!
//! Read in full (both the 219-line header and the 992-line source) against
//! the pinned SHA. Every symbol is classified below; see [`optimizer`]'s
//! own module doc for the full reasoning behind each classification.
//!
//! - Ported as free functions in [`optimizer`] (not as a `ChompOptimizer`
//!   struct — see [`optimizer`]'s module doc for why a faithful struct port
//!   is impossible in this crate): the inline `getPotential` →
//!   [`optimizer::get_potential`]; `calculateSmoothnessIncrements` →
//!   [`optimizer::calculate_smoothness_increments`];
//!   `calculateTotalIncrements` → [`optimizer::calculate_total_increments`]
//!   (this round's weighted-combination callout — see [`optimizer`]'s doc
//!   for the exact [`parameters::ChompParameters`] field-name mapping);
//!   `addIncrementsToTrajectory` → [`optimizer::add_increments_to_trajectory`];
//!   `getSmoothnessCost` → [`optimizer::get_smoothness_cost`];
//!   `handleJointLimits` → [`optimizer::handle_joint_limits`].
//! - Not ported, collision/kinematics-coupled: `ChompOptimizer` itself
//!   (struct, constructor, `optimize()`, `destroy()`, `isInitialized()`,
//!   `isCollisionFree()`), `performForwardKinematics`, `getCollisionCost`,
//!   `getTrajectoryCost`, `calculateCollisionIncrements`,
//!   `calculatePseudoInverse`, `getJacobian`, `computeJointProperties`,
//!   `setRobotStateFromPoint`, `registerParents`, the private `isParent`,
//!   `isCurrentTrajectoryMeshToMeshCollisionFree`. `optimize()`'s
//!   termination condition — this round's other callout — is transcribed as
//!   a specification (not executable code) in [`optimizer`]'s module doc.
//!   Round 18 re-checked this against `moveit-collision`/`moveit-distance-field`'s
//!   progress this session (per that round's Item 3): `GroupStateRepresentation`
//!   is now ported (`moveit_distance_field::GroupStateRepresentation`), which
//!   covers this struct's `gsr_` field, but `hy_env_`
//!   (`const collision_detection::CollisionEnvHybrid*`) has no path forward at
//!   all — `moveit-distance-field`'s own module doc lists `CollisionEnvHybrid`
//!   as a **whole-file exclusion**, D-decision: `PORTING-PLAN.md`'s FCL/Bullet
//!   → `parry3d-f64` backend replacement means `CollisionEnvHybrid` (which
//!   extends `CollisionEnvFCL` directly) is never ported, full stop, not
//!   "not yet". A literal port of this struct is therefore permanently
//!   impossible, not merely blocked on more infrastructure landing. A port
//!   *is* possible, but only by redesigning the collision-cost path against
//!   `moveit_collision::CollisionEnv<State>`/`moveit_scene::PlanningScene`'s
//!   generic environment parameter instead of upstream's concrete
//!   `CollisionEnvHybrid*` field — a semantic change to what "the same
//!   struct" means, per this port's own guidance on structural fixes needing
//!   sign-off before the rewrite, not after. Not attempted this round.
//! - Not ported, confirmed dead in upstream itself (not merely out of
//!   scope): `debugCost` (unused `std::cout` helper, no call site anywhere
//!   in `chomp_optimizer.cpp`); `perturbTrajectory`, `getRandomMomentum`,
//!   `updateMomentum`, `updatePositionFromMomentum` (the HMC path — every
//!   call site in `optimize()` is commented out, and the latter three have
//!   no implementation anywhere in the 992-line source at all, only header
//!   declarations).
//!
//! # Symbol audit: `multivariate_gaussian.hpp`
//!
//! - `MultivariateGaussian` (class) — ported as
//!   `moveit_sampling::`[`MultivariateGaussian`] (re-exported here), not a
//!   symbol local to this crate — see the "depends on `moveit-sampling`"
//!   section above for why. `sample` is ported as
//!   [`MultivariateGaussian::sample_with_covariance`]: CHOMP's `sample` has
//!   no `use_covariance` branch to reconcile (that is STOMP's sibling class),
//!   so it maps onto the one of `moveit-sampling`'s two named methods that
//!   always applies the covariance. See `moveit-sampling`'s own module doc
//!   for its full symbol mapping and deviations (fallible construction,
//!   `size_` derived rather than stored, etc.) — not re-documented here,
//!   since that crate owns the port.
//!
//! # Completion condition
//!
//! Stated as a check on the files audited so far, not a claim about the
//! crate: `chomp_parameters.{hpp,cpp}`, `chomp_utils.hpp`,
//! `chomp_trajectory.{hpp,cpp}`, `chomp_cost.{hpp,cpp}`,
//! `chomp_optimizer.{hpp,cpp}`, and `multivariate_gaussian.hpp` are each
//! read in full against the pinned SHA and every symbol in them is
//! classified above as ported (with its Rust name), D-decision-excluded
//! (with the decision), or confirmed dead upstream (with the evidence).
//! Phase 8's completion condition otherwise uses property-based
//! verification (`PORTING-PLAN.md` §5), not a trajectory oracle, and CHOMP
//! specifically is not the one Phase-8 planner (`moveit-planners-pilz`) with
//! directly comparable deterministic output — one op is the exception:
//! `chomp_quad_cost_inverse` (round 18, requested round 17 in
//! `doc/oracle-request-quad-cost-inv.md`), backing
//! `tests/chomp_quad_cost_inverse_parity.rs`'s direct element-by-element
//! comparison of [`cost::ChompCost::quadratic_cost_inverse`] against real
//! Eigen output, closing the decomposition-family gap round 16 could only
//! residual-check — see [`cost`]'s module doc for the measured numbers. What
//! else is pinned by unit test: [`trajectory::ChompTrajectory`]'s
//! copy-with-padding indexing/`full_trajectory_index_` convention,
//! [`cost::ChompCost`]'s finite-difference boundary truncation,
//! [`optimizer`]'s weighted-combination and joint-limit-repair formulas,
//! each checked against a hand-rolled recomputation of the same upstream
//! formula, not merely "it runs" — and
//! `moveit-sampling`'s own [`MultivariateGaussian`] shape/positive-definite
//! rejection and empirical mean/variance/correlation convergence (that
//! crate's tests, not this one's — see above). This
//! section does not cover `chomp_planner` — it is out of scope this round,
//! not implicitly satisfied by anything here.

/// `ChompParameters` and its trajectory-initialization-method validation —
/// see the module doc's `chomp_parameters.{hpp,cpp}` entry.
pub mod parameters;

/// `DIFF_RULE_LENGTH`/`DIFF_RULES` finite-difference stencils and CHOMP's own
/// angle-normalization helpers — see the module doc's `chomp_utils.hpp`
/// entry.
pub mod utils;

/// `ChompTrajectory` — see the module doc's `chomp_trajectory.{hpp,cpp}`
/// entry.
pub mod trajectory;

/// `ChompCost` — see the module doc's `chomp_cost.{hpp,cpp}` entry.
pub mod cost;

/// The portable numeric core of `ChompOptimizer` — see the module doc's
/// `chomp_optimizer.{hpp,cpp}` entry.
pub mod optimizer;

pub use cost::ChompCost;
pub use moveit_sampling::MultivariateGaussian;
pub use parameters::ChompParameters;
pub use trajectory::ChompTrajectory;
