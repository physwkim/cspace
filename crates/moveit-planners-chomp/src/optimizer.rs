// Copyright (c) 2009, Willow Garage, Inc.
// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// Ported from moveit2 @ e017c91ee12984393a28ba246075c65f69cde3bf:
//   moveit_planners/chomp/chomp_motion_planner/include/chomp_motion_planner/chomp_optimizer.hpp
//   moveit_planners/chomp/chomp_motion_planner/src/chomp_optimizer.cpp

//! The numeric core of `chomp::ChompOptimizer`, including the struct itself.
//!
//! # Round 19: `ChompOptimizer` is ported
//!
//! Round 17 classified `ChompOptimizer` (the struct, `optimize()`, and every
//! collision-coupled method) as permanently unportable, reasoning that its
//! only collision backend, `collision_detection::CollisionEnvHybrid`
//! (`hy_env_`), was a whole-file D-decision exclusion in
//! `moveit-distance-field` with no path forward at all. That reasoning did
//! not survive a direct read of `hy_env_`'s real use in
//! `chomp_motion_planner/`: it has exactly 5 references, and the only method
//! ever called on it, `getCollisionGradients`, is `CollisionEnvHybrid`'s own
//! one-line forward to `CollisionEnvDistanceField::getCollisionGradients` —
//! already ported as
//! [`moveit_distance_field::DistanceFieldCollisionCache::get_collision_gradients`].
//! `ChompOptimizer` never touches `CollisionEnvHybrid`'s FCL/Bullet-backed
//! narrow-phase at all; round 17 excluded the whole struct on the strength
//! of a field whose only real behavior was already portable.
//!
//! This round ports `ChompOptimizer` for real: the struct
//! ([`crate::optimizer::ChompOptimizer`]), its constructor ([`crate::optimizer::ChompOptimizer::new`]),
//! `optimize()` ([`crate::optimizer::ChompOptimizer::optimize`]), `isInitialized()`/
//! `isCollisionFree()` ([`crate::optimizer::ChompOptimizer::is_initialized`]/
//! [`crate::optimizer::ChompOptimizer::is_collision_free`]), and every method `optimize()`
//! calls transitively. [`crate::optimizer::ChompOptimizer`]'s own doc comment carries the full
//! list of deviations from upstream (external-resource-as-parameter instead
//! of stored `hy_env_`/`planning_scene_`/`full_trajectory_` fields, the
//! mesh-to-mesh check becoming an injected closure, the ancestor-query
//! collapse, the `Isometry3d * Vector3d` point-vs-vector transform, and
//! more) — not repeated here.
//!
//! The functions below `ChompOptimizer` in this file were already ported in
//! `b0e4826`, before `ChompOptimizer` (the class itself) was ported in
//! `77738b9`, when they were the only symbols judged portable at all (they
//! touch only already-ported types —
//! [`crate::trajectory::ChompTrajectory`], [`crate::cost::ChompCost`],
//! [`crate::parameters::ChompParameters`], `moveit_model`'s joint tree — and
//! needed no collision backend). Upstream declares each as a private
//! `ChompOptimizer` method reading `this`
//! (`chomp_optimizer.hpp:84,200,202,204,207,209`); they stay free functions
//! here rather than becoming `ChompOptimizer` methods retroactively, since
//! no call site needs that shape — every existing call site (including
//! `ChompOptimizer`'s own) already passes their state in explicitly, and
//! that remains true after `77738b9` added `ChompOptimizer` itself as a
//! caller.
//!
//! ## Precondition: `group_trajectory` must already be `DIFF_RULE_LENGTH`-padded
//!
//! Every function below that takes a `group_trajectory: &ChompTrajectory`
//! assumes its free block (`start_index()..=end_index()`, i.e.
//! [`ChompTrajectory::num_free_points`]) already coincides with the
//! `joint_costs`' own internal free-variable count
//! ([`ChompCost::quadratic_cost_inverse`]'s dimension). This is **not**
//! true of a trajectory built directly via
//! [`ChompTrajectory::from_num_points`] (default `start_index = 1`,
//! `end_index = num_points - 2`, so `num_free_points() = num_points - 2`);
//! it is only true once the trajectory has been padded via
//! [`ChompTrajectory::from_source_trajectory`] with
//! `diff_rule_length = `[`crate::utils::DIFF_RULE_LENGTH`] (`start_index =
//! DIFF_RULE_LENGTH - 1`, `end_index = num_points - 1 - (DIFF_RULE_LENGTH -
//! 1)`, so `num_free_points() = num_points - 2*(DIFF_RULE_LENGTH - 1)` —
//! exactly [`ChompCost::new`]'s own `num_vars_free` formula). Confirmed
//! against `chomp_optimizer.cpp:90-95`'s `initialize()`
//! (`num_vars_free_ = group_trajectory_.getNumFreePoints(); free_vars_start_
//! = group_trajectory_.getStartIndex(); free_vars_end_ =
//! group_trajectory_.getEndIndex();`) alongside `ChompCost`'s own
//! constructor: in this port, [`crate::optimizer::ChompOptimizer::new`]
//! (below) is the sole place `group_trajectory` is ever produced for
//! [`crate::optimizer::ChompOptimizer`] to hold — it always builds it via the padding
//! constructor ([`ChompTrajectory::from_source_trajectory`]) from the
//! caller's `full_trajectory` argument, regardless of whether that argument
//! was itself already padded, which is the only reason these two
//! independently-computed free-variable counts agree at all. (Upstream
//! guarantees the same thing one layer up instead: `chomp_planner.cpp`
//! always constructs its `group_trajectory_` via the padding constructor
//! before handing it to `ChompOptimizer`'s constructor, which then trusts
//! it rather than re-padding. This port's sole production call site,
//! [`crate::planner::solve`] (`chomp_planner.cpp:63-306`, ported), actually
//! passes an *unpadded* seed trajectory -- see `planner::build_seed_trajectory`
//! (private) -- relying on
//! [`crate::optimizer::ChompOptimizer::new`]'s own padding rather than reproducing upstream's
//! pre-padded-caller invariant. Should a future caller ever construct
//! `ChompOptimizer` by hand with a pre-padded `full_trajectory`, this still
//! holds: `from_source_trajectory` shrinks back to a no-op padding when the
//! source already has enough margin, per its own doc.) A plain
//! `from_num_points` trajectory passed as `group_trajectory` directly
//! (bypassing `ChompOptimizer::new`) has no such guarantee, and every
//! function below returns a typed error rather than a silently wrong answer
//! if `joint_costs`' dimension does not match `group_trajectory`'s free
//! block. Every test in this module builds its fixture through the padding
//! constructor for exactly this reason.
//!
//! ## Ported (this module)
//!
//! - `getPotential` (inline in the header) → [`crate::optimizer::get_potential`]. Pure
//!   3-branch scalar formula; no `ChompOptimizer` state at all.
//! - `calculateSmoothnessIncrements` → [`crate::optimizer::calculate_smoothness_increments`].
//!   Built entirely from [`crate::cost::ChompCost::derivative`] (already
//!   ported) and [`crate::trajectory::ChompTrajectory`] accessors.
//! - `calculateTotalIncrements` → [`crate::optimizer::calculate_total_increments`]. **This is
//!   the round's weighted-combination callout.** Upstream:
//!   `final_increments_.col(i) = learning_rate_ * (quad_cost_inv *
//!   (smoothness_cost_weight_ * smoothness_increments_.col(i) +
//!   obstacle_cost_weight_ * collision_increments_.col(i)))`. The three
//!   coefficients are, by name, exactly
//!   [`crate::parameters::ChompParameters::learning_rate`],
//!   [`crate::parameters::ChompParameters::smoothness_cost_weight`], and
//!   [`crate::parameters::ChompParameters::obstacle_cost_weight`] — all
//!   already-ported fields, confirmed against `chomp_optimizer.cpp`'s
//!   `calculateTotalIncrements` body directly (not inferred from the field
//!   names alone). This function takes `collision_increments` as a plain
//!   `&DMatrix<f64>` input rather than computing it internally, which is
//!   exactly what makes it portable: the formula does not care whether the
//!   caller's collision increments came from a real collision environment
//!   or a test fixture.
//! - `addIncrementsToTrajectory` → [`crate::optimizer::add_increments_to_trajectory`]. Pure
//!   per-joint scale-and-clamp against
//!   [`crate::parameters::ChompParameters::joint_update_limit`], writing
//!   through [`crate::trajectory::ChompTrajectory::free_trajectory_block_mut`].
//! - `getSmoothnessCost` → [`crate::optimizer::get_smoothness_cost`]. Sum of
//!   [`crate::cost::ChompCost::cost`] per joint, weighted by
//!   [`crate::parameters::ChompParameters::smoothness_cost_weight`] — the
//!   other half of the round's weight-mapping callout (the collision half,
//!   `getCollisionCost`'s `obstacle_cost_weight_`, is named in
//!   [`crate::optimizer::calculate_total_increments`]'s doc above since `getCollisionCost`
//!   itself is not portable — see below).
//! - `handleJointLimits` → [`crate::optimizer::handle_joint_limits`]. Needs only
//!   [`crate::trajectory::ChompTrajectory`], [`crate::cost::ChompCost`], and
//!   `moveit_model`'s joint bounds (`JointModel::variable_bounds`,
//!   `RevoluteJoint::is_continuous`) — all already dependencies of this
//!   crate, no collision environment involved.
//!
//! ## Ported as `ChompOptimizer` methods (`77738b9`)
//!
//! - `ChompOptimizer` (the class itself) and its constructor →
//!   [`crate::optimizer::ChompOptimizer`]/[`crate::optimizer::ChompOptimizer::new`]. `initialize()` upstream is
//!   not a separate step — folded into `new` the way
//!   [`crate::trajectory::ChompTrajectory`]'s own constructors already
//!   fold in upstream's private `init`.
//! - `optimize()` → [`crate::optimizer::ChompOptimizer::optimize`] — see the "termination
//!   condition" section below for the exact early-exit logic, now real,
//!   executed code rather than a specification.
//! - `isInitialized()`, `isCollisionFree()` →
//!   [`crate::optimizer::ChompOptimizer::is_initialized`]/[`crate::optimizer::ChompOptimizer::is_collision_free`].
//! - `performForwardKinematics` → [`crate::optimizer::ChompOptimizer::perform_forward_kinematics`]
//!   — populates every `collision_point_*` field via
//!   [`moveit_distance_field::DistanceFieldCollisionCache::get_collision_gradients`]
//!   (upstream: `hy_env_->getCollisionGradients(...)`); the only caller of
//!   [`crate::optimizer::get_potential`]. See the "closed API gap" section
//!   below for the `GradientInfo::sphere_locations` history this function's
//!   doc comment used to carry as a live deviation.
//! - `getCollisionCost`, `getTrajectoryCost` → private `get_collision_cost`/
//!   [`crate::optimizer::ChompOptimizer::get_trajectory_cost`] — read `collision_point_*`
//!   populated by `perform_forward_kinematics`.
//! - `calculateCollisionIncrements`, `calculatePseudoInverse`, `getJacobian`
//!   → private `calculate_collision_increments`/`calculate_pseudo_inverse`/
//!   `get_jacobian` — collision-point Jacobian computation.
//! - `computeJointProperties`, `setRobotStateFromPoint` → private
//!   `compute_joint_properties`/`set_robot_state_from_point` — forward
//!   kinematics against [`moveit_state::RobotState`], feeding
//!   `get_jacobian`.
//! - `registerParents`, the private `isParent` (inline) and
//!   `joint_parent_map_` → collapsed into one stateless helper,
//!   [`crate::optimizer::ChompOptimizer`]'s private `is_ancestor_or_self` — see
//!   [`crate::optimizer::ChompOptimizer`]'s own doc comment for the ancestor-resolution
//!   subtlety this collapse must still reproduce.
//! - `isCurrentTrajectoryMeshToMeshCollisionFree` → an injected
//!   `mesh_to_mesh_collision_free` closure parameter on
//!   [`crate::optimizer::ChompOptimizer::optimize`], not a method — see [`crate::optimizer::ChompOptimizer`]'s
//!   own doc comment for why (needs sign-off).
//!
//! ## Genuinely not ported
//!
//! - `destroy()` — upstream body is `{ // Nothing for now. }`
//!   (`chomp_optimizer.hpp:68-71`), an explicit no-op RAII hook; `Drop`
//!   makes it structurally unnecessary (PORTING-PLAN.md D1).
//! - `debugCost` — confirmed dead: not called anywhere in
//!   `chomp_optimizer.cpp` (a `std::cout` debug helper, same formula as
//!   [`crate::optimizer::get_smoothness_cost`] but unweighted and unused).
//! - `perturbTrajectory`, `getRandomMomentum`, `updateMomentum`,
//!   `updatePositionFromMomentum` (the HMC path) — confirmed dead in
//!   upstream itself, not merely out of this crate's scope: every call site
//!   in `optimize()` is commented out (`/// TODO: HMC BASED COMMENTED
//!   CODE...`), and `getRandomMomentum`/`updateMomentum`/
//!   `updatePositionFromMomentum` have **no implementation anywhere in
//!   `chomp_optimizer.cpp`** — declared in the header (lines 212-214) and
//!   never defined, which only compiles because nothing calls them.
//!   [`crate::optimizer::ChompOptimizer::new`] still constructs one
//!   [`moveit_sampling::MultivariateGaussian`] per joint (matching
//!   upstream's `multivariate_gaussian_.emplace_back(...)`, which also
//!   exists only to feed this dead path) but nothing ever samples it.
//! - `ChompPlanner` (the class itself, its ROS-typed
//!   `solve(PlanningContext, MotionPlanDetailedResponse)` entry point, and
//!   `PlanningContext` conformance) — excluded (D1): a ROS-facing wrapper
//!   with `moveit_msgs`-typed signatures throughout, none of which this
//!   crate depends on (see this crate's top-level module doc for the
//!   dependency check). Its model-independent numeric core, by contrast,
//!   *is* ported — as the free function [`crate::planner::solve`]
//!   (`eb4fa4e`); see this crate's top-level module doc's
//!   `chomp_planner.{hpp,cpp}` symbol audit for the full field-by-field
//!   account.
//!
//! ## Closed API gap: `GradientInfo::sphere_locations` (rounds 19-26)
//!
//! **Round 20 correction (`PORTING-PLAN.md` §154):** round 19 recorded this
//! gap's cause as "upstream only fills `sphere_locations` on the `gsr_`
//! reuse path, and this crate never stores `gsr_`". That upstream claim is
//! wrong, refuted by the oracle's own committed output (its
//! `group_state_representation_response.json` fixture calls
//! `getCollisionGradients` with a null `GroupStateRepresentationPtr` — not
//! the reuse path — yet returns non-empty `sphere_locations`, 1-9 entries
//! per link, summing to exactly `types`/`distances`' own length). The real
//! mechanism: `DistanceFieldCollisionCache::initialize()`
//! (`collision_env_distance_field.cpp:126`, called by **both**
//! constructors) walks every `JointModelGroup` and pre-builds a
//! `GroupStateRepresentation` for it up front (`:140-154`), stashing each
//! one on its `DistanceFieldCacheEntry`
//! (`pregenerated_group_state_representation_map_`, wired into
//! `dfce->pregenerated_group_state_representation_` at `:868-871`).
//! `getGroupStateRepresentation`'s truly-fresh branch (`:1161`) therefore
//! only ever executes *once per group, inside `initialize()` itself* — every
//! call after construction, reused-`gsr_` or not, takes the **pregenerated**
//! branch (`:1224`), which unconditionally sets `sphere_locations`
//! (`:1246` also sets it, unconditionally, for attached bodies). Whether the
//! caller keeps `gsr_` alive across `optimize()`'s loop is irrelevant to
//! upstream's own behavior; round 19's "gsr_-reuse-only" framing described a
//! mechanism upstream doesn't actually have.
//!
//! Round 20 concluded the gap was real for a different reason —
//! `moveit_distance_field::DistanceFieldCollisionCache` has no equivalent of
//! upstream's `initialize()` pregeneration step, so it only ever runs the
//! truly-fresh branch — and stated a falsifier: **expires once
//! `moveit-distance-field` builds a pregenerated `GroupStateRepresentation`
//! per `JointModelGroup` at cache-construction time, matching upstream's
//! `initialize()`.**
//!
//! **Round 26: closed, but not by that mechanism.** `moveit-distance-field`
//! round 25 (`f5328da`) did not port the cache-reuse/pregeneration
//! mechanism the falsifier above named — `GroupStateRepresentation` still
//! borrows its `dfce` rather than owning/sharing it, so a self-referential
//! pregenerated map would still need pinning/unsafe or an external crate
//! (see `moveit_distance_field::DistanceFieldCollisionCache::new`'s own doc
//! comment, which now carries that remaining, purely-performance gap and
//! its own falsifier). Instead, `group_state_representation`'s fresh-build
//! branch was changed to read `sphere_centers()` directly right after
//! posing each link's decomposition — closing the *value* gap without the
//! *mechanism* the falsifier predicted, because the pregenerated branch's
//! only field-level difference (`:1224`) never depended on which branch
//! computed it, only on the same posed decomposition both branches already
//! shared. The falsifier's predicted mechanism was wrong; the outcome it
//! was written to justify ("sphere_locations becomes reliable") arrived
//! anyway, by a simpler route this crate's own round-20 doc did not
//! consider. This crate's substitutions (sourcing sphere positions from
//! `link_body_decompositions[..].sphere_centers()` instead of
//! `sphere_locations`, and sizing per-link iteration from
//! `gradients.len()` instead of `sphere_locations.len()`) are removed in
//! `5293abd`: [`crate::optimizer::ChompOptimizer::perform_forward_kinematics`]
//! and the private `resolve_collision_point_joint_index` now read
//! `sphere_locations` directly, matching upstream's own indexing with no
//! substitution — see their own doc comments for the exact change, and
//! `get_collision_gradients_sphere_locations_matches_link_body_decompositions`
//! (this module's test suite) for the live-API proof that the substituted
//! and direct values were identical before the removal, so the removal
//! changes no computed output.
//!
//! ## `optimize()`'s termination condition
//!
//! Transcribed from `chomp_optimizer.cpp:290-518` and now real, executed
//! code ([`crate::optimizer::ChompOptimizer::optimize`]), pinned by
//! `optimize_runs_exactly_max_iterations_when_filter_mode_and_mesh_to_mesh_never_break_out`,
//! `optimize_breaks_out_immediately_when_max_iterations_after_collision_free_is_zero`,
//! and `optimize_collision_threshold_break_is_a_strict_less_than` in this
//! module's test suite. The loop runs `iteration_` from `0` to
//! [`crate::parameters::ChompParameters::max_iterations`] (exclusive),
//! computing `cost = collision_cost + smoothness_cost` each iteration and
//! tracking the minimum-cost trajectory seen (`best_group_trajectory_`,
//! restored at the very end regardless of how the loop exits — the
//! returned trajectory is never simply "the last iteration's"). Within an
//! iteration, three independent conditions can end the loop early, checked
//! in this order:
//!
//! 1. Every 10th iteration (`iteration_ % 10 == 0`), a full mesh-to-mesh
//!    collision check (upstream `isCurrentTrajectoryMeshToMeshCollisionFree`,
//!    here [`crate::optimizer::ChompOptimizer::optimize`]'s injected
//!    `mesh_to_mesh_collision_free` closure parameter)
//!    against the *current* trajectory (not `best_group_trajectory_`); a
//!    pass sets `num_collision_free_iterations_ = 0` (break on the very
//!    next check below).
//! 2. Unless [`crate::parameters::ChompParameters::filter_mode`] is set, the
//!    scalar comparison `collision_cost <
//!    `[`crate::parameters::ChompParameters::collision_threshold`]` — an
//!    **absolute**, not relative, comparison against one scalar (the raw
//!    collision cost, *not* `obstacle_cost_weight_ *` collision cost —
//!    confirmed from `getCollisionCost`'s call site in `optimize()`, which
//!    reads `c_cost` before the weight is applied inside `getCollisionCost`
//!    itself, i.e. `collision_threshold_` is compared in the *same*
//!    already-weighted units `getCollisionCost()` returns). On first
//!    satisfying this, `num_collision_free_iterations_` is set to
//!    [`crate::parameters::ChompParameters::max_iterations_after_collision_free`]
//!    — the optimizer keeps refining for up to that many further
//!    iterations rather than stopping immediately (immediately only when
//!    that parameter is `0`).
//! 3. A wall-clock check, `elapsed >`
//!    [`crate::parameters::ChompParameters::planning_time_limit`] — a plain
//!    `break`, no `should_break_out`/counter bookkeeping, checked
//!    unconditionally every iteration regardless of collision state.
//!
//! Once either early-exit condition (1) or (2) fires,
//! `collision_free_iteration_` increments every iteration thereafter, and
//! the loop breaks once it exceeds `num_collision_free_iterations_` (or
//! immediately if that count is `0`) — this is the "run a few more
//! iterations after first going collision-free, then stop" behavior.
//! `optimize()`'s boolean return is simply the final `is_collision_free_`
//! flag: `true` only if a collision-free trajectory was found before
//! `max_iterations_` ran out (or the wall clock did), regardless of the
//! final smoothness cost.

use crate::cost::ChompCost;
use crate::parameters::ChompParameters;
use crate::trajectory::ChompTrajectory;
use crate::utils::{DIFF_RULE_LENGTH, DIFF_RULES};
use moveit_collision::{AllowedCollisionMatrix, CollisionRequest};
use moveit_distance_field::{DistanceField, DistanceFieldCollisionCache, GradientInfo};
use moveit_error::{Error, Result};
use moveit_geometry::Vector3;
use moveit_model::joint::JointType;
use moveit_model::{JointModelGroup, RobotModel};
use moveit_state::RobotState;
use nalgebra::{DMatrix, Matrix3, Point3};
use rand::{Rng, RngExt};
use std::collections::HashMap;
use std::time::Instant;

/// The CHOMP potential function: a 3-branch scalar penalty for a collision
/// point at `field_distance` from the nearest obstacle, with a sphere of
/// `radius` and a desired clearance of `clearance`.
///
/// Ported from the private inline `getPotential`
/// (`chomp_optimizer.hpp:84-102`).
pub fn get_potential(field_distance: f64, radius: f64, clearance: f64) -> f64 {
    let d = field_distance - radius;
    if d >= clearance {
        0.0
    } else if d >= 0.0 {
        let diff = d - clearance;
        let gradient_magnitude = diff / clearance;
        0.5 * gradient_magnitude * diff
    } else {
        -d + 0.5 * clearance
    }
}

/// Computes the smoothness-cost increment for every joint and every free
/// trajectory point: `-derivative(joint_trajectory)` restricted to the free
/// (non-boundary-padding) segment, one column per joint.
///
/// `joint_costs[i]` must be the [`ChompCost`] for `group_trajectory`'s
/// joint column `i`; a length mismatch is a typed error rather than an
/// out-of-bounds panic.
///
/// Ported from `calculateSmoothnessIncrements`.
pub fn calculate_smoothness_increments(
    joint_costs: &[ChompCost],
    group_trajectory: &ChompTrajectory,
) -> Result<DMatrix<f64>> {
    let num_joints = group_trajectory.num_joints();
    if joint_costs.len() != num_joints {
        return Err(Error::other(format!(
            "joint_costs has {} entries, expected {num_joints}",
            joint_costs.len()
        )));
    }
    let num_vars_free = group_trajectory.num_free_points();
    let start_index = group_trajectory.start_index();
    let mut increments = DMatrix::<f64>::zeros(num_vars_free, num_joints);
    for (i, joint_cost) in joint_costs.iter().enumerate() {
        let derivative = joint_cost.derivative(&group_trajectory.joint_trajectory(i))?;
        for r in 0..num_vars_free {
            increments[(r, i)] = -derivative[start_index + r];
        }
    }
    Ok(increments)
}

/// Combines smoothness and collision increments into the final per-joint
/// trajectory update, weighted by
/// [`ChompParameters::smoothness_cost_weight`]/
/// [`ChompParameters::obstacle_cost_weight`] and scaled by
/// [`ChompParameters::learning_rate`] — see this module's doc comment for
/// the full field-name mapping.
///
/// `smoothness_increments` and `collision_increments` must have the same
/// shape (`num_vars_free` rows, `joint_costs.len()` columns); a mismatch
/// (either between the two matrices, or against `joint_costs`) is a typed
/// error.
///
/// Ported from `calculateTotalIncrements`.
pub fn calculate_total_increments(
    joint_costs: &[ChompCost],
    smoothness_increments: &DMatrix<f64>,
    collision_increments: &DMatrix<f64>,
    parameters: &ChompParameters,
) -> Result<DMatrix<f64>> {
    let num_joints = joint_costs.len();
    if smoothness_increments.ncols() != num_joints || collision_increments.ncols() != num_joints {
        return Err(Error::other(format!(
            "joint_costs has {num_joints} entries, but smoothness_increments has {} columns and collision_increments has {} columns",
            smoothness_increments.ncols(),
            collision_increments.ncols()
        )));
    }
    if smoothness_increments.nrows() != collision_increments.nrows() {
        return Err(Error::other(format!(
            "smoothness_increments has {} rows but collision_increments has {} rows",
            smoothness_increments.nrows(),
            collision_increments.nrows()
        )));
    }
    let num_vars_free = smoothness_increments.nrows();

    let mut final_increments = DMatrix::<f64>::zeros(num_vars_free, num_joints);
    for (i, joint_cost) in joint_costs.iter().enumerate() {
        let quad_cost_inv = joint_cost.quadratic_cost_inverse();
        if quad_cost_inv.nrows() != num_vars_free || quad_cost_inv.ncols() != num_vars_free {
            return Err(Error::other(format!(
                "joint {i}'s quadratic_cost_inverse is {}x{}, expected {num_vars_free}x{num_vars_free}",
                quad_cost_inv.nrows(),
                quad_cost_inv.ncols()
            )));
        }
        let combined = parameters.smoothness_cost_weight * smoothness_increments.column(i)
            + parameters.obstacle_cost_weight * collision_increments.column(i);
        final_increments.set_column(i, &(parameters.learning_rate * (quad_cost_inv * combined)));
    }
    Ok(final_increments)
}

/// Adds `final_increments` into `group_trajectory`'s free block, scaling
/// each joint's column so its largest-magnitude entry never exceeds
/// [`ChompParameters::joint_update_limit`].
///
/// `final_increments` must be `group_trajectory.num_free_points()` rows by
/// `group_trajectory.num_joints()` columns; a mismatch is a typed error.
/// Matches upstream's own IEEE-754 behavior on an all-zero column exactly:
/// `joint_update_limit / 0.0.abs()` is `+inf`, which never scales `scale`
/// down (`inf < 1.0` is `false`), the same outcome
/// `parameters_->joint_update_limit_ / fabs(0.0)` reaches upstream — not a
/// special case added here.
///
/// Ported from `addIncrementsToTrajectory`.
pub fn add_increments_to_trajectory(
    group_trajectory: &mut ChompTrajectory,
    final_increments: &DMatrix<f64>,
    joint_update_limit: f64,
) -> Result<()> {
    let num_joints = group_trajectory.num_joints();
    let num_vars_free = group_trajectory.num_free_points();
    if final_increments.nrows() != num_vars_free || final_increments.ncols() != num_joints {
        return Err(Error::other(format!(
            "final_increments is {}x{}, expected {num_vars_free}x{num_joints}",
            final_increments.nrows(),
            final_increments.ncols()
        )));
    }

    let mut block = group_trajectory.free_trajectory_block_mut();
    for i in 0..num_joints {
        let max = final_increments.column(i).max();
        let min = final_increments.column(i).min();
        let mut scale = 1.0f64;
        let max_scale = joint_update_limit / max.abs();
        let min_scale = joint_update_limit / min.abs();
        if max_scale < scale {
            scale = max_scale;
        }
        if min_scale < scale {
            scale = min_scale;
        }
        let mut col = block.column_mut(i);
        col += scale * final_increments.column(i);
    }
    Ok(())
}

/// The weighted sum of every joint's smoothness cost:
/// [`ChompParameters::smoothness_cost_weight`] times the sum of
/// [`ChompCost::cost`] over `joint_costs`/`group_trajectory`'s matching
/// joint columns.
///
/// Ported from `getSmoothnessCost`.
pub fn get_smoothness_cost(
    joint_costs: &[ChompCost],
    group_trajectory: &ChompTrajectory,
    smoothness_cost_weight: f64,
) -> Result<f64> {
    let mut smoothness_cost = 0.0;
    for (i, joint_cost) in joint_costs.iter().enumerate() {
        smoothness_cost += joint_cost.cost(&group_trajectory.joint_trajectory(i))?;
    }
    Ok(smoothness_cost_weight * smoothness_cost)
}

/// Repairs joint-limit violations in `group_trajectory`'s free block, one
/// joint at a time: up to 10 passes each pushing the single worst-violating
/// free trajectory point back toward its bound by
/// `violation / quad_cost_inv[(free_var_index, free_var_index)]` along
/// `quad_cost_inv`'s corresponding column (a smoothness-respecting
/// correction, not a hard clamp). Continuous revolute joints
/// ([`moveit_model::joint::RevoluteJoint::is_continuous`]) are skipped
/// entirely, matching upstream (a continuous joint has no bound to
/// violate).
///
/// `group`'s active joint count must equal `joint_costs.len()`; a mismatch
/// is a typed error rather than an out-of-bounds panic on either.
///
/// Ported from `handleJointLimits`.
pub fn handle_joint_limits(
    robot_model: &RobotModel,
    group: &JointModelGroup,
    group_trajectory: &mut ChompTrajectory,
    joint_costs: &[ChompCost],
) -> Result<()> {
    let active_joints = group.active_joint_indices();
    if active_joints.len() != joint_costs.len() {
        return Err(Error::other(format!(
            "group {:?} has {} active joints, but joint_costs has {} entries",
            group.name(),
            active_joints.len(),
            joint_costs.len()
        )));
    }

    let free_vars_start = group_trajectory.start_index();
    let free_vars_end = group_trajectory.end_index();

    for (joint_i, &model_index) in active_joints.iter().enumerate() {
        let joint_model = robot_model.joint_model_at(model_index);
        if joint_model.joint_type() == JointType::Revolute {
            if let Some(revolute) = joint_model.as_revolute() {
                if revolute.is_continuous() {
                    continue;
                }
            }
        }

        let mut joint_max = f64::MIN;
        let mut joint_min = f64::MAX;
        for bound in joint_model.variable_bounds() {
            if bound.min_position < joint_min {
                joint_min = bound.min_position;
            }
            if bound.max_position > joint_max {
                joint_max = bound.max_position;
            }
        }

        let mut count = 0;
        loop {
            let mut max_abs_violation = 1e-6;
            let mut max_violation = 0.0;
            let mut max_violation_index = free_vars_start;
            let mut violation = false;

            for i in free_vars_start..=free_vars_end {
                let value = group_trajectory[(i, joint_i)];
                let (amount, absolute_amount) = if value > joint_max {
                    let amount = joint_max - value;
                    (amount, amount.abs())
                } else if value < joint_min {
                    let amount = joint_min - value;
                    (amount, amount.abs())
                } else {
                    (0.0, 0.0)
                };
                if absolute_amount > max_abs_violation {
                    max_abs_violation = absolute_amount;
                    max_violation = amount;
                    max_violation_index = i;
                    violation = true;
                }
            }

            if violation {
                let free_var_index = max_violation_index - free_vars_start;
                let quad_cost_inv = joint_costs[joint_i].quadratic_cost_inverse();
                let multiplier = max_violation / quad_cost_inv[(free_var_index, free_var_index)];
                let update = quad_cost_inv.column(free_var_index) * multiplier;
                let mut block = group_trajectory.free_joint_trajectory_block_mut(joint_i);
                block += update;
            }

            count += 1;
            if count > 10 || !violation {
                break;
            }
        }
    }
    Ok(())
}

/// Borrows the two pieces of collision-checking state `ChompOptimizer`
/// needs from a caller-owned distance field, replacing upstream's
/// `hy_env_`/`planning_scene_` pair for the one call both of them actually
/// make: `getCollisionGradients`.
///
/// Neither field is stored on `ChompOptimizer` itself -- see the module
/// doc's "external-resource-as-parameter" note.
pub struct ChompCollisionContext<'a, 'm> {
    /// Upstream's `hy_env_`, narrowed to the one type it forwards to --
    /// see this module's doc comment for the `getCollisionGradients`
    /// one-line-forward evidence.
    pub cache: &'a mut DistanceFieldCollisionCache<'m>,
    /// The `PropagationDistanceField`-shaped backend `cache` checks
    /// against, threaded through the same way
    /// [`moveit_distance_field::DistanceFieldCollisionCache::get_collision_gradients`]
    /// itself requires.
    pub env_distance_field: &'a dyn DistanceField,
}

/// Builds, once per [`ChompOptimizer::new`] call, the reverse joint/link
/// lookups upstream reads directly off `JointModel::getParentLinkModel`/
/// `getChildLinkModel` -- neither is exposed by [`moveit_model::joint::JointModel`]
/// (it only carries a `parent_link_index` internally), so this port derives
/// both by scanning [`RobotModel::link_models`] once: a link's
/// [`moveit_model::LinkModel::parent_joint_index`] is that joint's *child*
/// link, and every entry of the link's own
/// [`moveit_model::LinkModel::child_joint_indices`] has that link as its
/// *parent* link.
///
/// Sized to `robot_model.joint_names().len()`; entries for joints outside
/// the group being optimized are simply never read.
fn joint_link_maps(robot_model: &RobotModel) -> (Vec<Option<usize>>, Vec<usize>) {
    let num_joints = robot_model.joint_names().len();
    let mut parent_link_of_joint = vec![None; num_joints];
    let mut child_link_of_joint = vec![0usize; num_joints];
    for (link_index, link) in robot_model.link_models().iter().enumerate() {
        child_link_of_joint[link.parent_joint_index()] = link_index;
        for &child_joint in link.child_joint_indices() {
            parent_link_of_joint[child_joint] = Some(link_index);
        }
    }
    (parent_link_of_joint, child_link_of_joint)
}

/// Ported from `initialize`'s `fixed_link_resolution_map` construction
/// (`chomp_optimizer.cpp:198-227`): resolves every joint a collision
/// gradient can be reported against to the joint whose Jacobian column it
/// should drive.
///
/// Transcribed as the exact three passes upstream runs, not collapsed into
/// one uniform walk -- they are NOT equivalent:
///
/// - Every active joint maps to itself.
/// - Every *fixed* joint maps to exactly **one** step up
///   (`getParentLinkModel()->getParentJointModel()`), regardless of whether
///   that lands on an active joint. If it doesn't, the fixed joint is left
///   resolved to a non-active joint permanently -- loop 3 below skips
///   anything already present in the map, so it never re-walks that entry
///   to find a real active ancestor.
/// - Every other "updated link" owner walks **multiple** steps up until it
///   reaches an active joint or the root.
///
/// This has an observable consequence reproduced deliberately, not fixed:
/// [`ChompOptimizer::is_ancestor_or_self`] (this port's `isParent`) treats a joint index
/// resolved to a non-active joint as having **no** registered ancestors at
/// all (upstream's `joint_parent_map_` is only ever populated by
/// `registerParents` for active joints, so `isParent`'s map lookup fails
/// silently for a non-active `childLink` and returns `false` for every
/// candidate) -- a collision point owned by such a joint contributes a
/// zero Jacobian column to every joint, discarding its gradient from the
/// optimization entirely.
///
/// Keyed and valued by joint index rather than upstream's joint name
/// (`std::map<std::string, std::string>`) -- indices are this port's
/// canonical joint identity throughout, avoiding a name lookup on every
/// resolution.
fn build_fixed_link_resolution_map(
    robot_model: &RobotModel,
    joint_model_group: &JointModelGroup,
    parent_link_of_joint: &[Option<usize>],
) -> HashMap<usize, usize> {
    let mut map = HashMap::new();
    for &active_idx in joint_model_group.active_joint_indices() {
        map.insert(active_idx, active_idx);
    }
    for &fixed_idx in joint_model_group.fixed_joint_indices() {
        let Some(parent_link) = parent_link_of_joint[fixed_idx] else {
            continue;
        };
        let one_up = robot_model.link_model_at(parent_link).parent_joint_index();
        map.insert(fixed_idx, one_up);
    }
    for &link_idx in joint_model_group.updated_link_indices() {
        let owner_joint = robot_model.link_model_at(link_idx).parent_joint_index();
        if map.contains_key(&owner_joint) {
            continue;
        }
        let mut parent_model = owner_joint;
        while let Some(parent_link) = parent_link_of_joint[parent_model] {
            parent_model = robot_model.link_model_at(parent_link).parent_joint_index();
            if joint_model_group
                .active_joint_indices()
                .contains(&parent_model)
            {
                break;
            }
        }
        map.insert(owner_joint, parent_model);
    }
    map
}

/// Resolves every collision gradient's `joint_name` (upstream
/// `GradientInfo::joint_name_`) to the joint index [`ChompOptimizer::get_jacobian`] should
/// treat it as owned by, via `resolution_map`
/// ([`build_fixed_link_resolution_map`]'s output). `None` if the name
/// isn't in `resolution_map` at all, matching upstream's
/// `RCLCPP_ERROR("Couldn't find joint %s!")` silent-failure path -- see
/// this function's caller for why an unresolved point is numerically
/// equivalent to upstream's default-constructed empty-string fallback.
///
/// Ported from the `for (i = free_vars_start_..=free_vars_end_)` /
/// `collision_point_joint_names_[i][j]` loop at the end of `initialize`
/// (`chomp_optimizer.cpp:229-247`). Flattened to one entry per collision
/// point rather than upstream's `num_vars_all_ x num_collision_points_`
/// grid: every row upstream writes is identical, since the loop body only
/// ever reads `gsr_->gradients_` -- a single snapshot taken once at
/// construction time, never re-read per trajectory point `i` in this
/// specific loop.
///
/// # Sized by `sphere_locations.len()`, matching upstream literally
///
/// Round 20 through round 25 gated this function's per-link entry count on
/// `gradients.len()` instead of upstream's own `for (k = 0; k <
/// info.sphere_locations.size(); ++k)`, because `GradientInfo::sphere_locations`
/// was empty for every link through this crate's only access path
/// ([`moveit_distance_field::DistanceFieldCollisionCache::get_collision_gradients`]'s
/// always-fresh-build) -- gating on it would have returned a zero-length
/// vector regardless of the real per-link sphere counts. `moveit-distance-field`
/// round 25 (`f5328da`) closed that gap in `group_state_representation`
/// itself, not by porting the cache-reuse mechanism the crate's own module
/// doc originally predicted as the closing condition, but by reading
/// `sphere_centers()` directly in the fresh-build branch -- the same value,
/// reached a different way (see this crate's module doc, "closed API gap"
/// section, for why the original falsifier prediction was wrong in its
/// mechanism but right in its outcome). `sphere_locations` is now sized
/// identically to `gradients`/`distances`/`sphere_radii` on every call
/// (proved directly against the live API by
/// `get_collision_gradients_sphere_locations_matches_link_body_decompositions`
/// in this module's test suite), so this function now matches upstream's
/// literal indexing with no substitution needed.
fn resolve_collision_point_joint_index(
    robot_model: &RobotModel,
    resolution_map: &HashMap<usize, usize>,
    gradients: &[GradientInfo],
) -> Vec<Option<usize>> {
    let mut collision_point_joint_index = Vec::new();
    for info in gradients {
        let joint_index = robot_model
            .joint_names()
            .iter()
            .position(|name| name == &info.joint_name);
        let resolved = joint_index.and_then(|idx| resolution_map.get(&idx).copied());
        for _ in 0..info.sphere_locations.len() {
            collision_point_joint_index.push(resolved);
        }
    }
    collision_point_joint_index
}

/// A single CHOMP optimization run over one planning group's trajectory.
///
/// Ported from `chomp::ChompOptimizer`.
///
/// # Deviations from upstream
///
/// - **No `hy_env_`/`planning_scene_`/`full_trajectory_` fields.** Upstream
///   stores the collision backend, planning scene, and the full-robot
///   trajectory it writes results back into as struct fields set once at
///   construction. This port threads all three through as explicit
///   borrowed parameters to [`ChompOptimizer::new`]/
///   [`ChompOptimizer::optimize`] instead, following the precedent already
///   established in `moveit-distance-field` (`env_distance_field: &dyn
///   DistanceField` is threaded the same way through
///   [`moveit_distance_field::DistanceFieldCollisionCache::get_collision_gradients`]
///   itself) -- this removes lifetime-parameter proliferation from the
///   struct (only `'m`, the robot model's lifetime, remains) at the cost
///   of a few more call-site arguments.
/// - **`gsr_`/`GroupStateRepresentation` is never stored.** Its accessor,
///   [`moveit_distance_field::DistanceFieldCollisionCache::get_collision_gradients`],
///   mutably borrows the collision cache for the `GroupStateRepresentation`'s
///   lifetime; storing it as a field would block every later mutable
///   reborrow of that same cache. Upstream's own usage never needs `gsr_`
///   to survive past copying its `gradients_` out into this type's
///   per-trajectory-point `Vec`s, so here it is always a function-local,
///   scoped to [`ChompOptimizer::new`]/
///   [`ChompOptimizer::perform_forward_kinematics`].
/// - **`isCurrentTrajectoryMeshToMeshCollisionFree` becomes an injected
///   closure**, not a method backed by `planning_scene_->isPathValid`.
///   **Round 20: approved** (`PORTING-PLAN.md` §154's review) --
///   wiring this as a method today would make `moveit-planners-chomp` --
///   per round 20's brief, and the `hy_env_`/`getCollisionGradients`
///   evidence backing it -- depend on two crates it has never carried:
///   `moveit-scene` (for `PlanningScene::is_path_valid`,
///   `scene.rs:1695`) and `moveit-collision` (for `ParryCollisionEnv`,
///   `parry.rs:1611` -- the only existing implementer of the
///   `CollisionEnv<Posed>` bound `is_path_valid` requires;
///   `DistanceFieldCollisionCache` does not implement it).
///   [`ChompOptimizer::optimize`] instead takes a
///   `mesh_to_mesh_collision_free: &mut dyn FnMut(&RobotState,
///   &DMatrix<f64>) -> bool` closure, called with `self.start_state` and
///   `self.best_group_trajectory` (matching upstream's own data source: the
///   check reads `best_group_trajectory_`'s *values* at
///   `group_trajectory_`'s *shape*, not the just-`updateFullTrajectory`'d
///   current iterate -- see `chomp_optimizer.cpp:520-537`).
///
///   **(a) What a caller passing `&mut |_, _| false` does not get.**
///   Upstream's early-exit condition 1 (the every-10th-iteration mesh check,
///   see this module's "termination condition" doc) becomes permanently
///   unreachable, so `is_collision_free_` can only ever become `true`
///   through condition 2, the sphere/distance-field-approximated
///   `collision_cost < collision_threshold_` comparison -- and only when
///   [`ChompParameters::filter_mode`] is unset. Two concrete upstream
///   behaviors are therefore lost, not merely "the mesh check is skipped":
///   (i) with `filter_mode` set, `optimize()` here can never report
///   collision-free early at all, only by exhausting `max_iterations`
///   (pinned by
///   `optimize_runs_exactly_max_iterations_when_filter_mode_and_mesh_to_mesh_never_break_out`);
///   (ii) even with `filter_mode` unset, a trajectory that is genuinely
///   mesh-collision-free but whose sphere-decomposition `collision_cost`
///   still sits at or above `collision_threshold_` -- a real case, since the
///   sphere decomposition is a padded over-approximation of the real mesh,
///   not merely a hypothetical one -- is caught early by upstream's mesh
///   check and is not caught early here.
///
///   **(b) What would need to exist for this to become a method.** The
///   underlying capability already exists elsewhere in this workspace --
///   `moveit_scene::PlanningScene::is_path_valid` is ported and generic over
///   `E: CollisionEnv<Posed>`, and `moveit_collision::ParryCollisionEnv`
///   already implements that bound. What is missing is specifically this
///   crate depending on both of them and threading a
///   `&mut PlanningScene`/`&ParryCollisionEnv` pair through
///   [`ChompOptimizer::optimize`] the same way [`ChompCollisionContext`]
///   already threads `DistanceFieldCollisionCache` -- at which point
///   `mesh_to_mesh_collision_free` collapses from an injected closure into a
///   real call to `scene.is_path_valid(env, request,
///   best_group_trajectory_as_states, path_constraints, goal_constraints)`.
///   Not attempted in `77738b9` (the commit that ported `ChompOptimizer`):
///   adding those two dependencies and the trajectory-to-`&[RobotState]`
///   conversion `is_path_valid` needs is a design decision of its own, not
///   implied by anything else that commit did. See (b) above for what
///   exactly would need to change for this to be attempted.
/// - **`dynamic_cast<const CollisionEnvHybrid*>` and its null check
///   disappear.** [`ChompCollisionContext::cache`] is already statically
///   typed as [`moveit_distance_field::DistanceFieldCollisionCache`]; Rust
///   has no equivalent of constructing a `ChompOptimizer` against a
///   differently-typed, incompatible collision environment for the caller
///   to fail at runtime.
/// - **`destroy()` is not ported.** Its upstream body is `{ // Nothing for
///   now. }` (`chomp_optimizer.hpp:68-71`) -- an explicit no-op RAII hook,
///   which `Drop` makes structurally unnecessary here (see PORTING-PLAN.md
///   D1).
/// - **The joint-ancestor query (`isParent`/`joint_parent_map_`/
///   `registerParents`) collapses into one stateless helper,
///   `ChompOptimizer::is_ancestor_or_self`**, computed by a direct
///   `RobotModel::parent_joint_index` chain-walk at call time rather than a
///   `HashMap` built once and consulted later. See this module's private
///   `build_fixed_link_resolution_map` for the one behavioral subtlety
///   this collapse must still reproduce (a resolved-to-non-active joint
///   has no ancestors at all, not "walk further to find one").
/// - **The `Eigen::Isometry3d * Eigen::Vector3d` in `computeJointProperties`'s
///   `axis = joint_transform * axis;` (`chomp_optimizer.cpp:733`) is ported
///   as a point transform, not a vector transform.** Eigen does not
///   distinguish "point" from "free vector" for a bare `Vector3d`, so that
///   multiplication applies the joint transform's translation *and*
///   rotation to `axis`. `nalgebra`'s `Isometry3<f64> * Vector3<f64>`
///   applies rotation only (nalgebra correctly distinguishes a `Vector3`
///   direction from a `Point3` position) -- a direct `joint_transform *
///   axis` port would silently drop the translation upstream actually
///   applies. This port instead computes `(joint_transform *
///   Point3::from(axis)).coords`, matching upstream's real (if unusual)
///   numeric behavior rather than the more "correct"-looking
///   rotation-only read of the same source line.
/// - **`rsl::uniform_real(0., 1.)` (`calculateCollisionIncrements`'s
///   stochastic-descent start point,
///   `chomp_motion_planner/src/chomp_optimizer.cpp:567`) becomes
///   `rng.random_range(0.0..1.0)`** on a caller-supplied `impl Rng`, the
///   same injected-RNG convention already established for
///   `moveit_sampling::MultivariateGaussian` in this crate; `rsl` itself is
///   not ported (D1: not a numeric-core dependency).
/// - **`calculateCollisionIncrements`'s two independent `should_break_out`
///   conditions in `optimize` (the `iteration_ % 10 == 0` mesh-to-mesh
///   check and the `!filter_mode_` collision-threshold check) are kept as
///   two separate, unconditionally-evaluated `if` blocks, not collapsed
///   into `if / else if`.** Both can fire in the same pass
///   (`chomp_optimizer.cpp:367-410`): if the first increments `iteration_`
///   and sets `num_collision_free_iterations_ = 0`, the second can
///   still fire afterward, incrementing `iteration_` a second time in one
///   loop pass and overwriting `num_collision_free_iterations_` with
///   `max_iterations_after_collision_free_`. This looks like it could be
///   an upstream bug, but the brief for this port is to transcribe the
///   numerics as written, not to "fix" behavior no test here contradicts.
/// - **Dead/write-only upstream fields are not ported at all**, verified
///   via `rg` across the whole `chomp_motion_planner` package, not just
///   `chomp_optimizer.cpp`: `group_trajectory_backup_` (read only inside
///   fully-commented-out HMC-perturbation code), `state_is_in_collision_`
///   (written every `performForwardKinematics` call, never read anywhere),
///   the *stored* `point_is_in_collision_` 2D field (its one read is
///   inside a `/* */`-commented block -- the *value* computed at
///   assignment time is still live, since it drives `is_collision_free_`,
///   so that computation survives as an inline local `bool` in
///   [`ChompOptimizer::perform_forward_kinematics`], just not as a
///   persisted field), and the entire dead-HMC-path field set already
///   established in round 18's [`crate::optimizer`] work
///   (`random_state_`, `joint_state_velocities_`, `momentum_`,
///   `random_momentum_`, `random_joint_momentum_`, `multivariate_gaussian_`,
///   `stochasticity_factor_`).
/// - **`smoothness_derivative_`/`jacobian_`/`jacobian_pseudo_inverse_`/
///   `jacobian_jacobian_tranpose_` are plain locals, not struct fields.**
///   Upstream's own header comment calls them "temporary variables for all
///   functions" (`chomp_optimizer.hpp:170`): every one is fully overwritten
///   before use in every call, so nothing is lost by not persisting them.
pub struct ChompOptimizer<'m> {
    num_joints: usize,
    num_vars_free: usize,
    num_vars_all: usize,
    num_collision_points: usize,
    free_vars_start: usize,
    free_vars_end: usize,
    iteration: i32,
    collision_free_iteration: u32,

    robot_model: &'m RobotModel,
    planning_group: String,
    parameters: ChompParameters,
    group_trajectory: ChompTrajectory,
    state: RobotState<'m>,
    start_state: RobotState<'m>,
    joint_model_group: &'m JointModelGroup,

    joint_costs: Vec<ChompCost>,
    initialized: bool,

    joint_names: Vec<String>,
    collision_point_joint_index: Vec<Option<usize>>,
    parent_link_of_joint: Vec<Option<usize>>,
    child_link_of_joint: Vec<usize>,

    collision_point_pos_eigen: Vec<Vec<Vector3>>,
    collision_point_vel_eigen: Vec<Vec<Vector3>>,
    collision_point_acc_eigen: Vec<Vec<Vector3>>,
    collision_point_potential: Vec<Vec<f64>>,
    collision_point_vel_mag: Vec<Vec<f64>>,
    collision_point_potential_gradient: Vec<Vec<Vector3>>,
    joint_axes: Vec<Vec<Vector3>>,
    joint_positions: Vec<Vec<Vector3>>,

    best_group_trajectory: DMatrix<f64>,
    best_group_trajectory_cost: f64,
    last_improvement_iteration: i32,
    num_collision_free_iterations: u32,

    is_collision_free: bool,
    worst_collision_cost_state: i64,
}

impl<'m> ChompOptimizer<'m> {
    /// Ported from the `ChompOptimizer` constructor plus `initialize`
    /// (`chomp_optimizer.cpp:63-247`). Unlike upstream, initialization
    /// cannot silently fail into an unusable, `is_initialized() == false`
    /// object: upstream's constructor returns early (leaving `initialized_
    /// == false`) only when `dynamic_cast<const CollisionEnvHybrid*>`
    /// fails, a case this port's static typing (see this type's
    /// "Deviations from upstream") makes unreachable, so every other
    /// upstream failure this constructor can hit (a missing group, a
    /// degenerate padded trajectory, a singular quadratic cost matrix) is
    /// a typed `Err` instead.
    pub fn new(
        full_trajectory: &ChompTrajectory,
        planning_group: &str,
        parameters: &ChompParameters,
        start_state: &RobotState<'m>,
        collision: &mut ChompCollisionContext<'_, 'm>,
        acm: Option<&AllowedCollisionMatrix>,
    ) -> Result<Self> {
        let robot_model = start_state.model();
        let joint_model_group = robot_model.joint_model_group(planning_group)?;
        let group_trajectory = ChompTrajectory::from_source_trajectory(
            full_trajectory,
            planning_group,
            DIFF_RULE_LENGTH,
        )?;

        let num_vars_free = group_trajectory.num_free_points();
        let num_vars_all = group_trajectory.num_points();
        let num_joints = group_trajectory.num_joints();
        let free_vars_start = group_trajectory.start_index();
        let free_vars_end = group_trajectory.end_index();

        let mut state = start_state.clone();
        let req = CollisionRequest {
            group_name: Some(planning_group.to_string()),
            ..CollisionRequest::default()
        };

        let (parent_link_of_joint, child_link_of_joint) = joint_link_maps(robot_model);
        let resolution_map =
            build_fixed_link_resolution_map(robot_model, joint_model_group, &parent_link_of_joint);

        let (num_collision_points, collision_point_joint_index) = {
            let posed = state.update();
            let gsr = collision.cache.get_collision_gradients(
                &req,
                &posed,
                acm,
                &[],
                collision.env_distance_field,
            )?;
            let num_collision_points = gsr.gradients.iter().map(|g| g.gradients.len()).sum();
            let collision_point_joint_index =
                resolve_collision_point_joint_index(robot_model, &resolution_map, &gsr.gradients);
            (num_collision_points, collision_point_joint_index)
        };

        // `joint_cost` is always 1.0: upstream's `nh.param("joint_costs/" +
        // name, joint_cost, 1.0)` ROS-param lookup is commented out
        // (`chomp_optimizer.cpp:112`), so `derivative_costs` is identical
        // for every joint and every `ChompCost` built below is identical
        // too -- upstream's own redundancy, kept faithfully rather than
        // deduplicated away.
        let mut joint_costs = Vec::with_capacity(num_joints);
        let mut max_cost_scale = 0.0f64;
        for _ in 0..num_joints {
            let derivative_costs = [
                parameters.smoothness_cost_velocity,
                parameters.smoothness_cost_acceleration,
                parameters.smoothness_cost_jerk,
            ];
            let cost = ChompCost::new(
                &group_trajectory,
                &derivative_costs,
                parameters.ridge_factor,
            )?;
            let scale = cost.max_quad_cost_inv_value()?;
            if scale > max_cost_scale {
                max_cost_scale = scale;
            }
            joint_costs.push(cost);
        }
        for cost in &mut joint_costs {
            cost.scale(max_cost_scale);
        }

        let best_group_trajectory = group_trajectory.trajectory_matrix().clone();

        let joint_names: Vec<String> = joint_model_group
            .active_joint_indices()
            .iter()
            .map(|&idx| robot_model.joint_model_at(idx).name().to_string())
            .collect();

        let zeros_vec3 = |n| vec![Vector3::zeros(); n];

        Ok(Self {
            num_joints,
            num_vars_free,
            num_vars_all,
            num_collision_points,
            free_vars_start,
            free_vars_end,
            iteration: 0,
            collision_free_iteration: 0,
            robot_model,
            planning_group: planning_group.to_string(),
            parameters: parameters.clone(),
            group_trajectory,
            state,
            start_state: start_state.clone(),
            joint_model_group,
            joint_costs,
            initialized: true,
            joint_names,
            collision_point_joint_index,
            parent_link_of_joint,
            child_link_of_joint,
            collision_point_pos_eigen: vec![zeros_vec3(num_collision_points); num_vars_all],
            collision_point_vel_eigen: vec![zeros_vec3(num_collision_points); num_vars_all],
            collision_point_acc_eigen: vec![zeros_vec3(num_collision_points); num_vars_all],
            collision_point_potential: vec![vec![0.0; num_collision_points]; num_vars_all],
            collision_point_vel_mag: vec![vec![0.0; num_collision_points]; num_vars_all],
            collision_point_potential_gradient: vec![
                zeros_vec3(num_collision_points);
                num_vars_all
            ],
            joint_axes: vec![zeros_vec3(num_joints); num_vars_all],
            joint_positions: vec![zeros_vec3(num_joints); num_vars_all],
            best_group_trajectory,
            best_group_trajectory_cost: 0.0,
            last_improvement_iteration: -1,
            num_collision_free_iterations: 0,
            is_collision_free: false,
            worst_collision_cost_state: -1,
        })
    }

    /// Ported from `isInitialized`.
    pub fn is_initialized(&self) -> bool {
        self.initialized
    }

    /// Ported from `isCollisionFree`.
    pub fn is_collision_free(&self) -> bool {
        self.is_collision_free
    }

    /// `isParent`, computed by chain-walk rather than a `HashMap` built by
    /// `registerParents` -- see this module's doc comment on
    /// `ChompOptimizer` and on [`build_fixed_link_resolution_map`] for the
    /// non-active-`child` quirk this must still reproduce.
    fn is_ancestor_or_self(&self, child: usize, candidate: usize) -> bool {
        if child == candidate {
            return true;
        }
        if !self
            .joint_model_group
            .active_joint_indices()
            .contains(&child)
        {
            return false;
        }
        let mut current = child;
        while let Some(parent) = self.robot_model.parent_joint_index(current) {
            if parent == candidate {
                return true;
            }
            current = parent;
        }
        false
    }

    /// Ported from `setRobotStateFromPoint`. Sets each active joint's
    /// single variable individually via
    /// [`moveit_state::RobotState::set_joint_positions`] rather than
    /// upstream's one batched `setJointGroupActivePositions` call -- this
    /// port's `RobotState` has no group-batched setter, and setting the
    /// same variables to the same values one joint at a time reaches the
    /// identical end state.
    fn set_robot_state_from_point(&mut self, trajectory_point: usize) {
        let point = self.group_trajectory.trajectory_point(trajectory_point);
        for (j, name) in self.joint_names.iter().enumerate() {
            // Every active joint has exactly 1 variable -- ChompTrajectory
            // itself enforces this (see its module doc's "Reachable
            // invariant violations" note), so this can't fail.
            self.state
                .set_joint_positions(name, &point[j..=j])
                .expect("ChompTrajectory guarantees every active joint has exactly 1 variable");
        }
    }

    /// Ported from `computeJointProperties`. See this type's doc comment
    /// for the `Eigen::Isometry3d * Eigen::Vector3d` point-vs-vector
    /// deviation this applies to `axis`.
    fn compute_joint_properties(&mut self, trajectory_point: usize) -> Result<()> {
        let posed = self.state.update();
        for j in 0..self.num_joints {
            let joint_index = self.joint_model_group.active_joint_indices()[j];
            let joint_model = self.robot_model.joint_model_at(joint_index);

            let parent_link = self.parent_link_of_joint[joint_index].ok_or_else(|| {
                Error::other(format!(
                    "joint {:?} has no parent link model; it cannot be the group's root joint",
                    joint_model.name()
                ))
            })?;

            let joint_origin_transform = *self
                .robot_model
                .link_model_at(self.child_link_of_joint[joint_index])
                .joint_origin_transform();
            let joint_transform = posed.global_link_transform_at(parent_link)
                * (joint_origin_transform * posed.joint_transform(joint_model.name())?);

            let axis = if let Some(revolute) = joint_model.as_revolute() {
                revolute.axis()
            } else if let Some(prismatic) = joint_model.as_prismatic() {
                prismatic.axis()
            } else {
                Vector3::new(1.0, 0.0, 0.0)
            };
            let axis = (joint_transform * Point3::from(axis)).coords;

            self.joint_axes[trajectory_point][j] = axis;
            self.joint_positions[trajectory_point][j] = joint_transform.translation.vector;
        }
        Ok(())
    }

    /// Ported from `performForwardKinematics` (`chomp_optimizer.cpp:862-940`).
    ///
    /// # `sphere_locations` gap closed (round 26)
    ///
    /// Rounds 20 through 25 read collision-point positions from
    /// `GroupStateRepresentation::link_body_decompositions[link_index].sphere_centers()`
    /// instead of `GradientInfo::sphere_locations`, because
    /// `DistanceFieldCollisionCache::get_collision_gradients` always takes
    /// the fresh-build path (no long-lived `gsr_` to reuse the way
    /// `chomp_optimizer.cpp` does), and that path left `sphere_locations`
    /// empty for every link. `moveit-distance-field` round 25 (`f5328da`)
    /// closed the gap directly in the fresh-build branch itself -- see
    /// `resolve_collision_point_joint_index`'s doc comment (private to this
    /// module) for the same fact and why the fix landed differently than
    /// this crate's own module doc originally predicted. `sphere_locations`
    /// is now read directly, matching upstream's own indexing
    /// (`info.sphere_locations[k]`,
    /// `collision_env_distance_field.cpp:920` in upstream's equivalent
    /// loop) with no substitution.
    ///
    /// This also removes a latent indexing hazard the old workaround
    /// carried: `link_body_decompositions` has one entry per
    /// `dfce.link_names` only, while `gradients` (what `link_index` here
    /// ranges over) has one entry per link *followed by* one per
    /// `dfce.attached_body_names` -- so `link_body_decompositions[link_index]`
    /// would have gone out of bounds for any attached-body gradient entry.
    /// Never triggered in practice (this crate always calls
    /// `get_collision_gradients` with `current_attached_bodies: &[]`, so
    /// `dfce.attached_body_names` stays empty), but `sphere_locations`
    /// indexes 1:1 with `gradients` unconditionally, so the hazard cannot
    /// recur regardless of that call-site fact holding.
    pub fn perform_forward_kinematics(
        &mut self,
        collision: &mut ChompCollisionContext<'_, 'm>,
    ) -> Result<()> {
        let inv_time = 1.0 / self.group_trajectory.discretization();
        let inv_time_sq = inv_time * inv_time;

        let (start, end) = if self.iteration == 0 {
            (0, self.num_vars_all - 1)
        } else {
            (self.free_vars_start, self.free_vars_end)
        };

        self.is_collision_free = true;

        let req = CollisionRequest {
            group_name: Some(self.planning_group.clone()),
            ..CollisionRequest::default()
        };

        for i in start..=end {
            self.set_robot_state_from_point(i);
            let posed = self.state.update();
            let gsr = collision.cache.get_collision_gradients(
                &req,
                &posed,
                None,
                &[],
                collision.env_distance_field,
            )?;

            let mut j = 0;
            for info in gsr.gradients.iter() {
                for k in 0..info.sphere_locations.len() {
                    if j >= self.num_collision_points {
                        return Err(Error::other(
                            "performForwardKinematics: gradients produced more collision points than new() found",
                        ));
                    }
                    self.collision_point_pos_eigen[i][j] = info.sphere_locations[k];
                    self.collision_point_potential[i][j] = get_potential(
                        info.distances[k],
                        info.sphere_radii[k],
                        self.parameters.min_clearance,
                    );
                    self.collision_point_potential_gradient[i][j] = info.gradients[k];

                    let point_is_in_collision =
                        info.distances[k] - info.sphere_radii[k] < info.sphere_radii[k];
                    if point_is_in_collision {
                        self.is_collision_free = false;
                    }
                    j += 1;
                }
            }
            drop(gsr);
            self.compute_joint_properties(i)?;
        }

        for i in self.free_vars_start..=self.free_vars_end {
            for j in 0..self.num_collision_points {
                let mut vel = Vector3::zeros();
                let mut acc = Vector3::zeros();
                for (k, (&d0, &d1)) in DIFF_RULES[0].iter().zip(DIFF_RULES[1].iter()).enumerate() {
                    let offset = k as i64 - (DIFF_RULE_LENGTH as i64 / 2);
                    let idx = (i as i64 + offset) as usize;
                    vel += (inv_time * d0) * self.collision_point_pos_eigen[idx][j];
                    acc += (inv_time_sq * d1) * self.collision_point_pos_eigen[idx][j];
                }
                self.collision_point_vel_eigen[i][j] = vel;
                self.collision_point_acc_eigen[i][j] = acc;
                self.collision_point_vel_mag[i][j] = vel.norm();
            }
        }
        Ok(())
    }

    /// `getJacobian`, restricted to `collision_point_joint_index`'s
    /// already-resolved owner (see [`resolve_collision_point_joint_index`]);
    /// an unresolved owner (`None`) produces an all-zero Jacobian, matching
    /// upstream's default-constructed-empty-string fallback (see that
    /// function's doc for why the two are numerically equivalent).
    fn get_jacobian(
        &self,
        trajectory_point: usize,
        collision_point_pos: &Vector3,
        owner_joint_index: Option<usize>,
    ) -> DMatrix<f64> {
        let mut jacobian = DMatrix::<f64>::zeros(3, self.num_joints);
        let Some(owner_joint_index) = owner_joint_index else {
            return jacobian;
        };
        for j in 0..self.num_joints {
            let candidate = self.joint_model_group.active_joint_indices()[j];
            if self.is_ancestor_or_self(owner_joint_index, candidate) {
                let column = self.joint_axes[trajectory_point][j]
                    .cross(&(collision_point_pos - self.joint_positions[trajectory_point][j]));
                jacobian[(0, j)] = column.x;
                jacobian[(1, j)] = column.y;
                jacobian[(2, j)] = column.z;
            }
        }
        jacobian
    }

    /// Ported from `calculatePseudoInverse`. `.inverse()`'s silent garbage
    /// on a singular matrix becomes a typed `Err`, matching this port's
    /// established `try_inverse()` convention (see [`crate::cost`]'s
    /// `calculate_pseudo_inverse` for the same substitution).
    fn calculate_pseudo_inverse(
        jacobian: &DMatrix<f64>,
        ridge_factor: f64,
    ) -> Result<DMatrix<f64>> {
        let jjt = jacobian * jacobian.transpose() + DMatrix::<f64>::identity(3, 3) * ridge_factor;
        let jjt_inv = jjt
            .try_inverse()
            .ok_or_else(|| Error::other("jacobian_jacobian_tranpose is singular"))?;
        Ok(jacobian.transpose() * jjt_inv)
    }

    /// Ported from `calculateCollisionIncrements`
    /// (`chomp_optimizer.cpp:548-623`). See this type's doc comment for the
    /// `rsl::uniform_real` -> `rng.random_range` substitution.
    fn calculate_collision_increments(&self, rng: &mut impl Rng) -> Result<DMatrix<f64>> {
        let mut collision_increments = DMatrix::<f64>::zeros(self.num_vars_free, self.num_joints);

        let (start_point, end_point) = if self.parameters.use_stochastic_descent {
            let raw = rng.random_range(0.0..1.0)
                * (self.free_vars_end as f64 - self.free_vars_start as f64)
                + self.free_vars_start as f64;
            let mut start_point = raw as i64;
            if start_point < self.free_vars_start as i64 {
                start_point = self.free_vars_start as i64;
            }
            if start_point > self.free_vars_end as i64 {
                start_point = self.free_vars_end as i64;
            }
            (start_point as usize, start_point as usize)
        } else {
            (self.free_vars_start, self.free_vars_end)
        };

        for i in start_point..=end_point {
            for j in 0..self.num_collision_points {
                let potential = self.collision_point_potential[i][j];
                if potential < 0.0001 {
                    continue;
                }
                let potential_gradient = -self.collision_point_potential_gradient[i][j];
                let vel = self.collision_point_vel_eigen[i][j];
                let vel_mag = self.collision_point_vel_mag[i][j];
                let vel_mag_sq = vel_mag * vel_mag;
                let normalized_velocity = vel / vel_mag;
                let orthogonal_projector = Matrix3::<f64>::identity()
                    - normalized_velocity * normalized_velocity.transpose();
                let curvature_vector =
                    (orthogonal_projector * self.collision_point_acc_eigen[i][j]) / vel_mag_sq;
                let cartesian_gradient = vel_mag
                    * (orthogonal_projector * potential_gradient - potential * curvature_vector);

                let owner = self.collision_point_joint_index[j];
                let jacobian = self.get_jacobian(i, &self.collision_point_pos_eigen[i][j], owner);

                let delta = if self.parameters.use_pseudo_inverse {
                    let pinv = Self::calculate_pseudo_inverse(
                        &jacobian,
                        self.parameters.pseudo_inverse_ridge_factor,
                    )?;
                    pinv * cartesian_gradient
                } else {
                    jacobian.transpose() * cartesian_gradient
                };
                let row = i - self.free_vars_start;
                for c in 0..self.num_joints {
                    collision_increments[(row, c)] -= delta[c];
                }
            }
        }
        Ok(collision_increments)
    }

    /// Ported from `getCollisionCost`. Weights every point's collision
    /// potential by `collision_point_vel_mag`, that point's velocity along
    /// the trajectory (`chomp_optimizer.cpp:942-963`) -- this is a *swept*
    /// cost, not a static-occupancy cost, so a perfectly stationary
    /// trajectory (identical start and goal) returns exactly `0.0` here
    /// regardless of how deeply it penetrates an obstacle. See
    /// `planner.rs`'s `solve_returns_invalid_motion_plan_when_the_path_cannot_escape_collision`
    /// doc comment for the consequence this has on `optimize()`'s
    /// collision-threshold branch.
    fn get_collision_cost(&mut self) -> f64 {
        let mut collision_cost = 0.0;
        let mut worst_collision_cost = 0.0;
        self.worst_collision_cost_state = -1;

        for i in self.free_vars_start..=self.free_vars_end {
            let mut state_collision_cost = 0.0;
            for j in 0..self.num_collision_points {
                state_collision_cost +=
                    self.collision_point_potential[i][j] * self.collision_point_vel_mag[i][j];
            }
            collision_cost += state_collision_cost;
            if state_collision_cost > worst_collision_cost {
                worst_collision_cost = state_collision_cost;
                self.worst_collision_cost_state = i as i64;
            }
        }
        self.parameters.obstacle_cost_weight * collision_cost
    }

    /// Ported from `getTrajectoryCost`.
    pub fn get_trajectory_cost(&mut self) -> Result<f64> {
        Ok(get_smoothness_cost(
            &self.joint_costs,
            &self.group_trajectory,
            self.parameters.smoothness_cost_weight,
        )? + self.get_collision_cost())
    }

    /// Ported from `ChompOptimizer::optimize` (`chomp_optimizer.cpp:289-518`).
    /// See this type's doc comment for the closure `mesh_to_mesh_collision_free`
    /// replaces, and for why its two `should_break_out` conditions are kept
    /// as two independent `if` blocks rather than collapsed.
    ///
    /// # Deviation (transcribed, not fixed): `self.iteration` can advance by
    /// 2 in a single pass
    ///
    /// Upstream's `for (iteration_ = 0; iteration_ < max_iterations_;
    /// ++iteration_)` (`chomp_optimizer.cpp:303`) increments `iteration_`
    /// unconditionally at the end of every pass, on top of the two
    /// branch-local `iteration_++;` at `:376` (the mesh check) and `:412`
    /// (the collision-threshold check) that each also fire when their
    /// condition is met. `should_break_out`'s gate (`:477-486`,
    /// `if (should_break_out) { collision_free_iteration_++; if
    /// (num_collision_free_iterations_ == 0) break; else if
    /// (collision_free_iteration_ > num_collision_free_iterations_) break;
    /// }`) does not `break` on every pass where `should_break_out` was set --
    /// only once the grace period (`num_collision_free_iterations_`, from
    /// `parameters_->max_iterations_after_collision_free_`) is exhausted.
    /// When it doesn't break, control falls through to the `for` loop's own
    /// `++iteration_` -- so that pass advances `iteration_` by 2, not 1. The
    /// mesh-check branch forecloses this on its own (it hardcodes
    /// `num_collision_free_iterations_ = 0`, which always breaks
    /// immediately), but the collision-threshold branch sets
    /// `num_collision_free_iterations_` to the configured grace period,
    /// which is `5` by default (`ChompParameters::default`) -- so with
    /// default parameters, the very first pass where `c_cost` drops below
    /// `collision_threshold_` is exactly such a pass, transcribed here as
    /// `self.iteration += 1` inside the `if !self.parameters.filter_mode &&
    /// c_cost < self.parameters.collision_threshold` block (below) plus the
    /// unconditional `self.iteration += 1` at this loop's end. This is
    /// reachable on ordinary inputs, not a corner case requiring a crafted
    /// fixture -- see
    /// `optimize_reaches_iteration_two_after_one_pass_via_the_double_increment`
    /// in this module's tests, which pins it with `max_iterations: 1` and
    /// otherwise-default parameters: exactly one real optimization pass
    /// runs, and `self.iteration` ends at `2`, not `1`. Left as upstream
    /// wrote it (`PORTING-PLAN.md`: transcribe the numerics, don't rewrite
    /// them into something cleaner) -- this changes how many grace-period
    /// passes actually execute before `num_collision_free_iterations_` is
    /// exhausted (each such pass consumes 2 of `max_iterations_`'s budget
    /// instead of 1), not whether the loop terminates or what it returns.
    pub fn optimize(
        &mut self,
        full_trajectory: &mut ChompTrajectory,
        collision: &mut ChompCollisionContext<'_, 'm>,
        mesh_to_mesh_collision_free: &mut dyn FnMut(&RobotState<'m>, &DMatrix<f64>) -> bool,
        rng: &mut impl Rng,
    ) -> Result<bool> {
        let start_time = Instant::now();

        self.iteration = 0;
        while self.iteration < self.parameters.max_iterations {
            self.perform_forward_kinematics(collision)?;
            let c_cost = self.get_collision_cost();
            let s_cost = get_smoothness_cost(
                &self.joint_costs,
                &self.group_trajectory,
                self.parameters.smoothness_cost_weight,
            )?;
            let cost = c_cost + s_cost;

            if self.iteration == 0 || cost < self.best_group_trajectory_cost {
                self.best_group_trajectory = self.group_trajectory.trajectory_matrix().clone();
                self.best_group_trajectory_cost = cost;
                self.last_improvement_iteration = self.iteration;
            }

            let smoothness_increments =
                calculate_smoothness_increments(&self.joint_costs, &self.group_trajectory)?;
            let collision_increments = self.calculate_collision_increments(rng)?;
            let final_increments = calculate_total_increments(
                &self.joint_costs,
                &smoothness_increments,
                &collision_increments,
                &self.parameters,
            )?;
            add_increments_to_trajectory(
                &mut self.group_trajectory,
                &final_increments,
                self.parameters.joint_update_limit,
            )?;

            handle_joint_limits(
                self.robot_model,
                self.joint_model_group,
                &mut self.group_trajectory,
                &self.joint_costs,
            )?;
            full_trajectory.update_from_group_trajectory(&self.group_trajectory);

            let mut should_break_out = false;

            if self.iteration % 10 == 0
                && mesh_to_mesh_collision_free(&self.start_state, &self.best_group_trajectory)
            {
                self.num_collision_free_iterations = 0;
                self.is_collision_free = true;
                self.iteration += 1;
                should_break_out = true;
            }

            if !self.parameters.filter_mode && c_cost < self.parameters.collision_threshold {
                self.num_collision_free_iterations =
                    self.parameters.max_iterations_after_collision_free as u32;
                self.is_collision_free = true;
                self.iteration += 1;
                should_break_out = true;
            }

            if start_time.elapsed().as_secs_f64() > self.parameters.planning_time_limit {
                break;
            }

            if should_break_out {
                self.collision_free_iteration += 1;
                // Both upstream arms are a bare `break;` (the second also guards
                // dead, commented-out logging) -- one condition reaches the same
                // outcome without an empty branch for clippy::if_same_then_else.
                if self.num_collision_free_iterations == 0
                    || self.collision_free_iteration > self.num_collision_free_iterations
                {
                    break;
                }
            }

            self.iteration += 1;
        }

        let optimization_result = self.is_collision_free;

        for i in 0..self.num_vars_all {
            let row: Vec<f64> = self.best_group_trajectory.row(i).iter().copied().collect();
            self.group_trajectory.set_trajectory_point(i, &row);
        }
        full_trajectory.update_from_group_trajectory(&self.group_trajectory);

        Ok(optimization_result)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;
    use moveit_model::MeshSearchPaths;
    use moveit_srdf::SrdfModel;
    use std::sync::OnceLock;

    const EPS: f64 = 1e-12;
    const GROUP: &str = "panda_arm";

    fn panda_model() -> &'static RobotModel {
        static MODEL: OnceLock<RobotModel> = OnceLock::new();
        MODEL.get_or_init(|| {
            let urdf_path = concat!(env!("CARGO_MANIFEST_DIR"), "/../../fixtures/panda.urdf");
            let srdf_path = concat!(env!("CARGO_MANIFEST_DIR"), "/../../fixtures/panda.srdf");
            let urdf_xml = std::fs::read_to_string(urdf_path)
                .unwrap_or_else(|e| panic!("read {urdf_path}: {e}"));
            let urdf = urdf_rs::read_file(urdf_path).expect("panda.urdf parses");
            let srdf = SrdfModel::parse_file(srdf_path).expect("panda.srdf parses");
            RobotModel::from_urdf_and_srdf(&urdf, &urdf_xml, &srdf, &MeshSearchPaths::none())
                .expect("panda model builds")
        })
    }

    /// Builds a `DIFF_RULE_LENGTH`-padded trajectory (see this module's
    /// "Precondition" doc section) with `source_num_points` points before
    /// padding.
    fn trajectory(source_num_points: usize) -> ChompTrajectory {
        let source = ChompTrajectory::from_num_points(panda_model(), source_num_points, 0.1, GROUP)
            .expect("valid num_points");
        ChompTrajectory::from_source_trajectory(&source, GROUP, crate::utils::DIFF_RULE_LENGTH)
            .expect("valid padding")
    }

    fn joint_costs(traj: &ChompTrajectory, ridge_factor: f64) -> Vec<ChompCost> {
        (0..traj.num_joints())
            .map(|_| ChompCost::new(traj, &[0.0, 1.0, 0.0], ridge_factor).unwrap())
            .collect()
    }

    // --- ChompOptimizer ---

    use moveit_collision::LinkPaddingScale;
    use moveit_distance_field::{
        DistanceFieldConfig, GridGeometry, PropagationDistanceField, add_link_body_decompositions,
    };
    use rand::SeedableRng;
    use rand_chacha::ChaCha8Rng;

    const CHOMP_COLLISION_GROUP: &str = "chain";

    /// A synthetic two-joint revolute chain with primitive (`<box>`)
    /// collision geometry, matching the construction idiom
    /// `moveit-distance-field`'s own `get_collision_gradients` tests use
    /// (`two_link_model_and_srdf` in
    /// `collision_env_distance_field.rs`). `panda.urdf`'s `<collision>`
    /// tags are all `<mesh>` references, which `MeshSearchPaths::none()`
    /// (this crate's own test setup) skips entirely per
    /// `moveit_model::MeshSearchPaths::none`'s own doc comment -- so
    /// `panda_model()` has zero collision spheres for every link, and the
    /// tests below need real spheres. Unlike `two_link_model_and_srdf`
    /// (whose `mid`/`tip` are deliberately coincident, for *self*-collision
    /// tests), each joint here carries a `0.3 0 0` `<origin>`, spacing
    /// `base`/`mid`/`tip`'s 0.1 m collision boxes well apart at the default
    /// pose -- `get_collision_gradients` runs self- and intra-group
    /// proximity before the environment check
    /// (`collision_env_distance_field.rs`'s own `get_collision_gradients`),
    /// and only overwrites `GradientInfo::distances[i]` when a later check
    /// finds something *closer*; a coincident fixture's self-collision
    /// distance (~0) would permanently win over this crate's own
    /// environment-obstacle tests, which need the environment distance to
    /// be the one that lands in `distances[i]`. The smoothness/free-function
    /// tests above this section are unaffected and keep using
    /// `panda_model()`.
    fn chomp_collision_model() -> RobotModel {
        let urdf_xml = r#"<?xml version="1.0"?>
<robot name="two_link_chomp">
  <link name="base">
    <collision><geometry><box size="0.1 0.1 0.1"/></geometry></collision>
  </link>
  <link name="mid">
    <collision><geometry><box size="0.1 0.1 0.1"/></geometry></collision>
  </link>
  <link name="tip">
    <collision><geometry><box size="0.1 0.1 0.1"/></geometry></collision>
  </link>
  <joint name="j1" type="revolute">
    <parent link="base"/>
    <child link="mid"/>
    <origin xyz="0.3 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1" upper="1" effort="1" velocity="1"/>
  </joint>
  <joint name="j2" type="revolute">
    <parent link="mid"/>
    <child link="tip"/>
    <origin xyz="0.3 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1" upper="1" effort="1" velocity="1"/>
  </joint>
</robot>
"#;
        let srdf_xml = r#"<?xml version="1.0"?>
<robot name="two_link_chomp">
  <group name="chain">
    <chain base_link="base" tip_link="tip"/>
  </group>
</robot>
"#;
        let urdf: urdf_rs::Robot = urdf_rs::read_from_string(urdf_xml).unwrap();
        let srdf = SrdfModel::parse_str(srdf_xml).expect("srdf must parse");
        let model =
            RobotModel::from_urdf_and_srdf(&urdf, urdf_xml, &srdf, &MeshSearchPaths::none())
                .expect("two_link_chomp model must build");
        // PORTING-PLAN.md §196: an SRDF chain group over a fixed joint
        // resolves to `updated_link_names() == []` with no error and no
        // warning, so every test built on this fixture would pass
        // vacuously -- the checks this fixture feeds
        // (`CollisionRequest::group_name` above, line 1747) resolve through
        // that set, not the raw `link_names()`/`joint_names()` topology
        // (see `ParryCollisionEnv::active_group_links`). Both `j1` and `j2`
        // above are `revolute`, not `fixed`, but assert the group actually
        // has updated links rather than trusting that stays true.
        moveit_test_support::assert_group_has_updated_links(&model, CHOMP_COLLISION_GROUP);
        model
    }

    /// A grid covering `chomp_collision_model`'s 0.6 m-long chain, at a
    /// coarser resolution than production for test speed -- these tests
    /// exercise `ChompOptimizer`'s collision-gradient plumbing, not
    /// distance-field accuracy.
    fn chomp_collision_field_config() -> DistanceFieldConfig {
        let size = Vector3::new(3.0, 3.0, 3.0);
        let origin_center = Vector3::new(0.0, 0.0, 0.0);
        DistanceFieldConfig {
            geometry: GridGeometry::new(size, origin_center - 0.5 * size, 0.02).unwrap(),
            max_propagation_distance: 0.3,
            use_signed_distance_field: false,
        }
    }

    fn chomp_collision_cache(model: &RobotModel) -> DistanceFieldCollisionCache<'_> {
        let padding = LinkPaddingScale::new();
        let decompositions = add_link_body_decompositions(model, 0.02, &padding, None).unwrap();
        DistanceFieldCollisionCache::new(decompositions, chomp_collision_field_config(), 0.0)
    }

    fn env_field_with_points(points: &[Vector3]) -> PropagationDistanceField {
        let config = chomp_collision_field_config();
        let mut field = PropagationDistanceField::new(
            config.geometry,
            config.max_propagation_distance,
            config.use_signed_distance_field,
        )
        .unwrap();
        if !points.is_empty() {
            field.add_points_to_field(points);
        }
        field
    }

    /// An unpadded source trajectory -- [`ChompOptimizer::new`] pads it
    /// internally via [`ChompTrajectory::from_source_trajectory`], unlike
    /// this module's other tests which pad up front to exercise the free
    /// functions directly.
    fn chomp_full_trajectory(model: &RobotModel, num_points: usize) -> ChompTrajectory {
        ChompTrajectory::from_num_points(model, num_points, 0.1, CHOMP_COLLISION_GROUP)
            .expect("valid num_points")
    }

    /// Proves the invariant [`ChompOptimizer::perform_forward_kinematics`]
    /// and [`resolve_collision_point_joint_index`] now rely on directly,
    /// rather than working around: as of `moveit-distance-field` round 25
    /// (`f5328da`), every [`GradientInfo`] this crate's only access path
    /// (`DistanceFieldCollisionCache::get_collision_gradients`) returns has
    /// `sphere_locations` populated, and it is element-for-element identical
    /// to `link_body_decompositions[i].sphere_centers()` -- the exact value
    /// the pre-round-26 workaround read instead. If `moveit-distance-field`
    /// ever regresses `sphere_locations` back to empty (or lets it diverge
    /// from `link_body_decompositions`), this test fails first, before
    /// [`ChompOptimizer::perform_forward_kinematics`]'s own tests would
    /// (whose obstacle-placement assertions are self-consistent regardless
    /// of which array positions are sourced from, so cannot by themselves
    /// catch a sourcing regression).
    #[test]
    fn get_collision_gradients_sphere_locations_matches_link_body_decompositions() {
        let model = chomp_collision_model();
        let mut cache = chomp_collision_cache(&model);
        let field = env_field_with_points(&[]);
        let mut start_state = RobotState::new(&model);
        let posed = start_state.update();
        let req = CollisionRequest {
            group_name: Some(CHOMP_COLLISION_GROUP.to_string()),
            ..CollisionRequest::default()
        };

        let gsr = cache
            .get_collision_gradients(&req, &posed, None, &[], &field)
            .unwrap();

        assert_eq!(
            gsr.link_body_decompositions.len(),
            gsr.gradients.len(),
            "chomp_collision_model has no attached bodies, so gradients must have exactly one \
             entry per link, matching link_body_decompositions 1:1"
        );
        let mut checked_a_geometry_bearing_link = false;
        for (link_index, link_bd) in gsr.link_body_decompositions.iter().enumerate() {
            let Some(link_bd) = link_bd else { continue };
            checked_a_geometry_bearing_link = true;
            let info = &gsr.gradients[link_index];
            assert_eq!(
                info.sphere_locations.len(),
                info.gradients.len(),
                "sphere_locations must be sized identically to gradients/distances/sphere_radii, \
                 since group_state_representation pushes all of them from the same sphere_count"
            );
            assert_eq!(
                info.sphere_locations,
                link_bd.sphere_centers(),
                "sphere_locations must equal link_body_decompositions[..].sphere_centers() -- the \
                 value the pre-round-26 workaround read instead of sphere_locations"
            );
        }
        assert!(
            checked_a_geometry_bearing_link,
            "chomp_collision_model's chain group must have at least one geometry-bearing link"
        );
    }

    #[test]
    fn perform_forward_kinematics_reports_collision_free_with_no_obstacle_in_env_field() {
        let model = chomp_collision_model();
        let source = chomp_full_trajectory(&model, 10);
        let start_state = RobotState::new(&model);
        let mut cache = chomp_collision_cache(&model);
        let field = env_field_with_points(&[]);
        let mut collision = ChompCollisionContext {
            cache: &mut cache,
            env_distance_field: &field,
        };
        let parameters = ChompParameters::default();
        let mut optimizer = ChompOptimizer::new(
            &source,
            CHOMP_COLLISION_GROUP,
            &parameters,
            &start_state,
            &mut collision,
            None,
        )
        .expect("ChompOptimizer::new succeeds");
        assert!(
            optimizer.num_collision_points > 0,
            "the chain group must have at least one collision sphere"
        );

        optimizer
            .perform_forward_kinematics(&mut collision)
            .unwrap();

        assert!(optimizer.is_collision_free());
        assert_relative_eq!(optimizer.get_collision_cost(), 0.0, epsilon = EPS);
    }

    #[test]
    fn perform_forward_kinematics_flags_the_point_an_obstacle_sits_on() {
        let model = chomp_collision_model();
        let source = chomp_full_trajectory(&model, 10);
        let start_state = RobotState::new(&model);
        let parameters = ChompParameters::default();

        let obstacle_point = {
            let mut cache = chomp_collision_cache(&model);
            let field = env_field_with_points(&[]);
            let mut collision = ChompCollisionContext {
                cache: &mut cache,
                env_distance_field: &field,
            };
            let mut optimizer = ChompOptimizer::new(
                &source,
                CHOMP_COLLISION_GROUP,
                &parameters,
                &start_state,
                &mut collision,
                None,
            )
            .unwrap();
            optimizer
                .perform_forward_kinematics(&mut collision)
                .unwrap();
            assert!(optimizer.num_collision_points > 0);
            optimizer.collision_point_pos_eigen[optimizer.free_vars_start][0]
        };

        let mut cache = chomp_collision_cache(&model);
        let field = env_field_with_points(&[obstacle_point]);
        let mut collision = ChompCollisionContext {
            cache: &mut cache,
            env_distance_field: &field,
        };
        let mut optimizer = ChompOptimizer::new(
            &source,
            CHOMP_COLLISION_GROUP,
            &parameters,
            &start_state,
            &mut collision,
            None,
        )
        .unwrap();
        optimizer
            .perform_forward_kinematics(&mut collision)
            .unwrap();

        assert!(!optimizer.is_collision_free());
        assert!(
            optimizer.collision_point_potential[optimizer.free_vars_start][0] > 0.0,
            "an obstacle placed exactly on this collision sphere's center must register nonzero potential"
        );
    }

    #[test]
    fn get_trajectory_cost_is_smoothness_only_when_collision_cost_is_zero() {
        let model = chomp_collision_model();
        let source = chomp_full_trajectory(&model, 10);
        let start_state = RobotState::new(&model);
        let mut cache = chomp_collision_cache(&model);
        let field = env_field_with_points(&[]);
        let mut collision = ChompCollisionContext {
            cache: &mut cache,
            env_distance_field: &field,
        };
        let parameters = ChompParameters::default();
        let mut optimizer = ChompOptimizer::new(
            &source,
            CHOMP_COLLISION_GROUP,
            &parameters,
            &start_state,
            &mut collision,
            None,
        )
        .unwrap();
        optimizer
            .perform_forward_kinematics(&mut collision)
            .unwrap();

        let expected_smoothness = get_smoothness_cost(
            &optimizer.joint_costs,
            &optimizer.group_trajectory,
            parameters.smoothness_cost_weight,
        )
        .unwrap();

        assert_relative_eq!(
            optimizer.get_trajectory_cost().unwrap(),
            expected_smoothness,
            epsilon = EPS
        );
    }

    #[test]
    fn optimize_runs_exactly_max_iterations_when_filter_mode_and_mesh_to_mesh_never_break_out() {
        let model = chomp_collision_model();
        let source = chomp_full_trajectory(&model, 10);
        let start_state = RobotState::new(&model);
        let mut full = source.clone();
        let mut cache = chomp_collision_cache(&model);
        let field = env_field_with_points(&[]);
        let mut collision = ChompCollisionContext {
            cache: &mut cache,
            env_distance_field: &field,
        };
        let parameters = ChompParameters {
            max_iterations: 3,
            filter_mode: true,
            ..ChompParameters::default()
        };
        let mut optimizer = ChompOptimizer::new(
            &source,
            CHOMP_COLLISION_GROUP,
            &parameters,
            &start_state,
            &mut collision,
            None,
        )
        .unwrap();
        let mut rng = ChaCha8Rng::seed_from_u64(1);

        optimizer
            .optimize(&mut full, &mut collision, &mut |_, _| false, &mut rng)
            .unwrap();

        assert_eq!(
            optimizer.iteration, 3,
            "filter_mode disables the only reachable should_break_out path, so the loop must run to max_iterations"
        );
    }

    #[test]
    fn optimize_breaks_out_immediately_when_max_iterations_after_collision_free_is_zero() {
        let model = chomp_collision_model();
        let source = chomp_full_trajectory(&model, 10);
        let start_state = RobotState::new(&model);
        let mut full = source.clone();
        let mut cache = chomp_collision_cache(&model);
        let field = env_field_with_points(&[]);
        let mut collision = ChompCollisionContext {
            cache: &mut cache,
            env_distance_field: &field,
        };
        let parameters = ChompParameters {
            max_iterations: 50,
            max_iterations_after_collision_free: 0,
            ..ChompParameters::default()
        };
        let mut optimizer = ChompOptimizer::new(
            &source,
            CHOMP_COLLISION_GROUP,
            &parameters,
            &start_state,
            &mut collision,
            None,
        )
        .unwrap();
        let mut rng = ChaCha8Rng::seed_from_u64(1);

        let result = optimizer
            .optimize(&mut full, &mut collision, &mut |_, _| false, &mut rng)
            .unwrap();

        assert!(result, "an empty env field never puts a point in collision");
        assert_eq!(
            optimizer.iteration, 1,
            "num_collision_free_iterations == 0 must break out on the very first should_break_out pass"
        );
        assert!(optimizer.iteration < parameters.max_iterations);
    }

    #[test]
    fn optimize_collision_threshold_break_is_a_strict_less_than() {
        let model = chomp_collision_model();
        let source = chomp_full_trajectory(&model, 10);
        let start_state = RobotState::new(&model);

        // Measure the actual collision cost an empty env field produces --
        // expected to be exactly 0.0, but measured rather than assumed.
        let measured_c_cost = {
            let mut cache = chomp_collision_cache(&model);
            let field = env_field_with_points(&[]);
            let mut collision = ChompCollisionContext {
                cache: &mut cache,
                env_distance_field: &field,
            };
            let parameters = ChompParameters::default();
            let mut optimizer = ChompOptimizer::new(
                &source,
                CHOMP_COLLISION_GROUP,
                &parameters,
                &start_state,
                &mut collision,
                None,
            )
            .unwrap();
            optimizer
                .perform_forward_kinematics(&mut collision)
                .unwrap();
            optimizer.get_collision_cost()
        };

        let run = |collision_threshold: f64| -> i32 {
            let mut full = source.clone();
            let mut cache = chomp_collision_cache(&model);
            let field = env_field_with_points(&[]);
            let mut collision = ChompCollisionContext {
                cache: &mut cache,
                env_distance_field: &field,
            };
            let parameters = ChompParameters {
                max_iterations: 3,
                max_iterations_after_collision_free: 0,
                collision_threshold,
                ..ChompParameters::default()
            };
            let mut optimizer = ChompOptimizer::new(
                &source,
                CHOMP_COLLISION_GROUP,
                &parameters,
                &start_state,
                &mut collision,
                None,
            )
            .unwrap();
            let mut rng = ChaCha8Rng::seed_from_u64(1);
            optimizer
                .optimize(&mut full, &mut collision, &mut |_, _| false, &mut rng)
                .unwrap();
            optimizer.iteration
        };

        assert_eq!(
            run(measured_c_cost),
            3,
            "c_cost < collision_threshold must be strict: c_cost == threshold must not break out, \
             so the loop runs to max_iterations"
        );
        assert_eq!(
            run(measured_c_cost + f64::EPSILON.max(1e-12)),
            1,
            "a threshold strictly above the measured c_cost must break out on the first pass"
        );
    }

    /// Pins `optimize`'s doc comment's "`self.iteration` can advance by 2 in
    /// a single pass" deviation note. Unlike
    /// `optimize_collision_threshold_break_is_a_strict_less_than` (which
    /// forces `max_iterations_after_collision_free: 0` to get an immediate
    /// break), this uses `ChompParameters::default`'s own grace period
    /// (`5`), so `num_collision_free_iterations` is non-zero when the
    /// collision-threshold branch trips -- `collision_free_iteration`
    /// reaches only `1`, `1 > 5` is false, and the loop does not break: the
    /// branch-local `self.iteration += 1` and the loop's own unconditional
    /// `self.iteration += 1` both apply on this one pass.
    #[test]
    fn optimize_reaches_iteration_two_after_one_pass_via_the_double_increment() {
        let model = chomp_collision_model();
        let source = chomp_full_trajectory(&model, 10);
        let start_state = RobotState::new(&model);
        let mut full = source.clone();
        let mut cache = chomp_collision_cache(&model);
        let field = env_field_with_points(&[]);
        let mut collision = ChompCollisionContext {
            cache: &mut cache,
            env_distance_field: &field,
        };
        let parameters = ChompParameters {
            max_iterations: 1,
            ..ChompParameters::default()
        };
        assert!(
            !parameters.filter_mode && parameters.max_iterations_after_collision_free > 0,
            "the double increment needs the threshold branch reachable (filter_mode off) \
             and its grace period non-zero (or the threshold branch's should_break_out \
             breaks immediately instead of falling through to the loop's own increment)"
        );
        let mut optimizer = ChompOptimizer::new(
            &source,
            CHOMP_COLLISION_GROUP,
            &parameters,
            &start_state,
            &mut collision,
            None,
        )
        .unwrap();
        let mut rng = ChaCha8Rng::seed_from_u64(1);

        optimizer
            .optimize(&mut full, &mut collision, &mut |_, _| false, &mut rng)
            .unwrap();

        assert_eq!(
            optimizer.iteration, 2,
            "an empty env field's c_cost (0.0) is below the default collision_threshold \
             (0.07) from the first pass, so the threshold branch's should_break_out fires \
             on iteration 0 without breaking (grace period 5 > collision_free_iteration 1) -- \
             the pass's branch-local increment plus the loop's own unconditional increment \
             put self.iteration at 2 after exactly one real optimization pass, not 1"
        );
    }

    // get_potential: one case per branch plus both boundaries.
    #[test]
    fn get_potential_is_zero_at_and_beyond_clearance() {
        assert_relative_eq!(get_potential(1.0, 0.0, 0.2), 0.0, epsilon = EPS);
        assert_relative_eq!(get_potential(2.0, 0.0, 0.2), 0.0, epsilon = EPS);
    }

    #[test]
    fn get_potential_transition_branch_matches_closed_form() {
        // d = field_distance - radius = 0.1, clearance = 0.2: transition
        // branch. diff = 0.1-0.2 = -0.1, gradient = -0.5, 0.5*-0.5*-0.1 =
        // 0.025.
        let got = get_potential(0.3, 0.2, 0.2);
        assert_relative_eq!(got, 0.025, epsilon = EPS);
    }

    #[test]
    fn get_potential_at_d_equals_zero_matches_both_adjacent_branches() {
        // d == 0.0 exactly: takes the transition branch (d >= 0.0), and by
        // continuity must equal the collision branch's limit as d -> 0^-.
        let clearance = 0.4;
        let transition = get_potential(0.5, 0.5, clearance);
        assert_relative_eq!(transition, 0.5 * clearance, epsilon = EPS);
    }

    #[test]
    fn get_potential_collision_branch_matches_closed_form() {
        // d = -0.05, clearance = 0.2: collision branch, -d + 0.5*clearance.
        let got = get_potential(0.15, 0.2, 0.2);
        assert_relative_eq!(got, 0.05 + 0.1, epsilon = EPS);
    }

    #[test]
    fn calculate_smoothness_increments_rejects_joint_costs_length_mismatch() {
        let traj = trajectory(20);
        let costs = joint_costs(&traj, 1e-6);
        let err = calculate_smoothness_increments(&costs[..costs.len() - 1], &traj).unwrap_err();
        assert!(matches!(err, Error::Other(_)));
    }

    #[test]
    fn calculate_smoothness_increments_is_negative_gradient_restricted_to_free_rows() {
        let traj = trajectory(20);
        let costs = joint_costs(&traj, 1e-6);
        let increments = calculate_smoothness_increments(&costs, &traj).unwrap();
        assert_eq!(increments.nrows(), traj.num_free_points());
        assert_eq!(increments.ncols(), traj.num_joints());

        let derivative = costs[0].derivative(&traj.joint_trajectory(0)).unwrap();
        for r in 0..traj.num_free_points() {
            assert_relative_eq!(
                increments[(r, 0)],
                -derivative[traj.start_index() + r],
                epsilon = EPS,
                max_relative = EPS
            );
        }
    }

    #[test]
    fn calculate_total_increments_matches_hand_rolled_weighted_combination() {
        let traj = trajectory(20);
        let costs = joint_costs(&traj, 1e-6);
        let num_free = traj.num_free_points();
        let num_joints = traj.num_joints();

        let smoothness = DMatrix::<f64>::from_fn(num_free, num_joints, |r, c| (r + c) as f64);
        let collision = DMatrix::<f64>::from_fn(num_free, num_joints, |r, c| (r * c) as f64 * 0.1);

        let parameters = ChompParameters {
            smoothness_cost_weight: 0.3,
            obstacle_cost_weight: 0.7,
            learning_rate: 0.05,
            ..ChompParameters::default()
        };

        let got = calculate_total_increments(&costs, &smoothness, &collision, &parameters).unwrap();

        let expected_col0 = parameters.learning_rate
            * (costs[0].quadratic_cost_inverse()
                * (parameters.smoothness_cost_weight * smoothness.column(0)
                    + parameters.obstacle_cost_weight * collision.column(0)));
        for r in 0..num_free {
            assert_relative_eq!(
                got[(r, 0)],
                expected_col0[r],
                epsilon = EPS,
                max_relative = EPS
            );
        }
    }

    #[test]
    fn calculate_total_increments_rejects_column_count_mismatch() {
        let traj = trajectory(20);
        let costs = joint_costs(&traj, 1e-6);
        let num_free = traj.num_free_points();
        let smoothness = DMatrix::<f64>::zeros(num_free, costs.len());
        let collision = DMatrix::<f64>::zeros(num_free, costs.len() - 1);
        let parameters = ChompParameters::default();
        let err =
            calculate_total_increments(&costs, &smoothness, &collision, &parameters).unwrap_err();
        assert!(matches!(err, Error::Other(_)));
    }

    #[test]
    fn add_increments_to_trajectory_clamps_to_joint_update_limit() {
        let mut traj = trajectory(20);
        let num_free = traj.num_free_points();
        let num_joints = traj.num_joints();
        // Column 0 has max entry 10.0 -> scale = joint_update_limit / 10.0.
        let mut final_increments = DMatrix::<f64>::zeros(num_free, num_joints);
        final_increments[(0, 0)] = 10.0;
        final_increments[(1, 0)] = -2.0;

        let joint_update_limit = 0.1;
        add_increments_to_trajectory(&mut traj, &final_increments, joint_update_limit).unwrap();

        let scale = joint_update_limit / 10.0;
        assert_relative_eq!(
            traj[(traj.start_index(), 0)],
            scale * 10.0,
            epsilon = EPS,
            max_relative = EPS
        );
        assert_relative_eq!(
            traj[(traj.start_index() + 1, 0)],
            scale * -2.0,
            epsilon = EPS,
            max_relative = EPS
        );
    }

    #[test]
    fn add_increments_to_trajectory_rejects_shape_mismatch() {
        let mut traj = trajectory(20);
        let wrong = DMatrix::<f64>::zeros(traj.num_free_points() - 1, traj.num_joints());
        let err = add_increments_to_trajectory(&mut traj, &wrong, 0.1).unwrap_err();
        assert!(matches!(err, Error::Other(_)));
    }

    #[test]
    fn get_smoothness_cost_matches_weighted_sum_of_per_joint_cost() {
        let traj = trajectory(20);
        let costs = joint_costs(&traj, 1e-6);
        let weight = 0.4;
        let got = get_smoothness_cost(&costs, &traj, weight).unwrap();

        let mut expected = 0.0;
        for (i, joint_cost) in costs.iter().enumerate() {
            expected += joint_cost.cost(&traj.joint_trajectory(i)).unwrap();
        }
        expected *= weight;
        assert_relative_eq!(got, expected, epsilon = EPS, max_relative = EPS);
    }

    #[test]
    fn handle_joint_limits_pulls_an_out_of_bounds_point_back_toward_the_bound() {
        let model = panda_model();
        let group = model.joint_model_group(GROUP).unwrap();
        let mut traj = trajectory(20);
        let costs = joint_costs(&traj, 1e-3);

        let joint_model = model.joint_model_at(group.active_joint_indices()[0]);
        let joint_max = joint_model
            .variable_bounds()
            .iter()
            .map(|b| b.max_position)
            .fold(f64::MIN, f64::max);

        let mid = (traj.start_index() + traj.end_index()) / 2;
        let violating = joint_max + 10.0;
        let mut row = traj.trajectory_point(mid);
        row[0] = violating;
        traj.set_trajectory_point(mid, &row);

        handle_joint_limits(model, group, &mut traj, &costs).unwrap();

        assert!(
            traj[(mid, 0)] < violating,
            "expected handle_joint_limits to reduce the violating value, got {}",
            traj[(mid, 0)]
        );
    }

    #[test]
    fn handle_joint_limits_rejects_joint_costs_length_mismatch() {
        let model = panda_model();
        let group = model.joint_model_group(GROUP).unwrap();
        let mut traj = trajectory(20);
        let costs = joint_costs(&traj, 1e-3);
        let err =
            handle_joint_limits(model, group, &mut traj, &costs[..costs.len() - 1]).unwrap_err();
        assert!(matches!(err, Error::Other(_)));
    }

    #[test]
    fn handle_joint_limits_is_a_noop_when_no_bound_is_violated() {
        let model = panda_model();
        let group = model.joint_model_group(GROUP).unwrap();
        let mut traj = trajectory(20);
        let costs = joint_costs(&traj, 1e-3);
        let before = traj.clone();
        handle_joint_limits(model, group, &mut traj, &costs).unwrap();
        assert_eq!(traj, before);
    }
}
