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
//! `cspace-distance-field` with no path forward at all. That reasoning did
//! not survive a direct read of `hy_env_`'s real use in
//! `chomp_motion_planner/`: it has exactly 5 references, and the only method
//! ever called on it, `getCollisionGradients`, is `CollisionEnvHybrid`'s own
//! one-line forward to `CollisionEnvDistanceField::getCollisionGradients` —
//! already ported as
//! [`cspace_collision::distance_field::DistanceFieldCollisionCache::get_collision_gradients`].
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
//! [`crate::parameters::ChompParameters`], `cspace_core::model`'s joint tree — and
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
//!   `cspace_core::model`'s joint bounds (`JointModel::variable_bounds`,
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
//!   [`cspace_collision::distance_field::DistanceFieldCollisionCache::get_collision_gradients`]
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
//!   kinematics against [`cspace_core::state::RobotState`], feeding
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
//!   [`cspace_core::sampling::MultivariateGaussian`] per joint (matching
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
//! `cspace_collision::distance_field::DistanceFieldCollisionCache` has no equivalent of
//! upstream's `initialize()` pregeneration step, so it only ever runs the
//! truly-fresh branch — and stated a falsifier: **expires once
//! `cspace-distance-field` builds a pregenerated `GroupStateRepresentation`
//! per `JointModelGroup` at cache-construction time, matching upstream's
//! `initialize()`.**
//!
//! **Round 26: closed, but not by that mechanism.** `cspace-distance-field`
//! round 25 (`f5328da`) did not port the cache-reuse/pregeneration
//! mechanism the falsifier above named — `GroupStateRepresentation` still
//! borrows its `dfce` rather than owning/sharing it, so a self-referential
//! pregenerated map would still need pinning/unsafe or an external crate
//! (see `cspace_collision::distance_field::DistanceFieldCollisionCache::new`'s own doc
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
//!    `mesh_to_mesh_collision_free` closure parameter) against
//!    `best_group_trajectory_`'s *values* (at `group_trajectory_`'s *shape*),
//!    not the just-`updateFullTrajectory`'d current iterate -- despite its
//!    name, `isCurrentTrajectoryMeshToMeshCollisionFree` never reads
//!    `group_trajectory_`'s values (`chomp_optimizer.cpp:520-537`); a pass
//!    sets `num_collision_free_iterations_ = 0` (break on the very next
//!    check below).
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
use cspace_collision::distance_field::{DistanceField, DistanceFieldCollisionCache, GradientInfo};
use cspace_collision::{AllowedCollisionMatrix, CollisionRequest};
use cspace_core::error::{Error, Result};
use cspace_core::geometry::Vector3;
use cspace_core::model::joint::JointType;
use cspace_core::model::{JointModelGroup, RobotModel};
use cspace_core::state::RobotState;
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
/// Returns the largest absolute per-entry change it applied, which is the
/// scaled quantity and therefore the one a caller measuring "did the update
/// collapse" has to read. Deriving it outside would mean re-deriving `scale`
/// against the same three-way `min`, i.e. a second implementation of the rule
/// this function owns; upstream discards it (its own `// ROS_DEBUG("Scale:
/// %f",scale)` at
/// `moveit_planners/chomp/chomp_motion_planner/src/chomp_optimizer.cpp:659` is
/// commented out).
///
/// Ported from `addIncrementsToTrajectory`.
pub fn add_increments_to_trajectory(
    group_trajectory: &mut ChompTrajectory,
    final_increments: &DMatrix<f64>,
    joint_update_limit: f64,
) -> Result<f64> {
    let num_joints = group_trajectory.num_joints();
    let num_vars_free = group_trajectory.num_free_points();
    if final_increments.nrows() != num_vars_free || final_increments.ncols() != num_joints {
        return Err(Error::other(format!(
            "final_increments is {}x{}, expected {num_vars_free}x{num_joints}",
            final_increments.nrows(),
            final_increments.ncols()
        )));
    }

    let mut applied_max = 0.0f64;
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
        let update = scale * final_increments.column(i);
        applied_max = applied_max.max(update.amax());
        let mut col = block.column_mut(i);
        col += update;
    }
    Ok(applied_max)
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
/// ([`cspace_core::model::joint::RevoluteJoint::is_continuous`]) are skipped
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

/// One evaluation of CHOMP's objective function, kept as its two terms
/// rather than their sum.
///
/// # Deviation: upstream keeps this value, and emits it in exactly one shape
///
/// Upstream computes both terms every iteration (`chomp_optimizer.cpp:306-308`,
/// `c_cost`/`s_cost`) and stores their sum on the private member
/// `best_group_trajectory_cost_` (`chomp_optimizer.hpp:150`), whose three
/// accessors -- `getTrajectoryCost`, `getSmoothnessCost`, `getCollisionCost`
/// (`chomp_optimizer.hpp:208-210`) -- are all below that header's `private:`
/// at `:83`. `ChompOptimizer::optimize` returns a bare `bool`, and
/// `chomp_planner.cpp` never mentions cost at all, so no caller of upstream's
/// planner can reach the number.
///
/// The one place upstream lets it out is a log line:
/// `RCLCPP_DEBUG(getLogger(), "Collision cost %f, smoothness cost: %f",
/// c_cost, s_cost)` (`chomp_optimizer.cpp:310`). That line is the whole
/// justification for this type's shape -- **the two terms, separately**, in
/// upstream's own vocabulary, because a single scalar is a granularity
/// upstream never makes visible. Upstream logs it per iteration and never
/// logs the sum, nor the cost of the trajectory it actually returns.
///
/// Carrying it out of `solve` is therefore a real deviation, recorded on
/// [`crate::planner::ChompSolution::objective`]. What is *not* invented here
/// is the decomposition or the word `best`: both come from the lines above.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ChompObjective {
    /// Upstream's `s_cost` -- `getSmoothnessCost()` at
    /// `chomp_optimizer.cpp:307`, already weighted by
    /// `smoothness_cost_weight`.
    pub smoothness: f64,
    /// Upstream's `c_cost` -- `getCollisionCost()` at
    /// `chomp_optimizer.cpp:306`, already weighted by `obstacle_cost_weight`.
    pub collision: f64,
}

impl ChompObjective {
    /// The value upstream compares and stores: `c_cost + s_cost`
    /// (`chomp_optimizer.cpp:308`), which is also what `getTrajectoryCost`
    /// returns (`chomp_optimizer.cpp:678`).
    #[must_use]
    pub fn total(&self) -> f64 {
        self.smoothness + self.collision
    }
}

/// The objective at the two points that make a *paired* improvement claim
/// possible: the trajectory CHOMP was handed, and the trajectory it returns.
///
/// Both are upstream quantities read at upstream's own moments. `seed` is the
/// cost computed on iteration 0, before any increment has been applied --
/// upstream's `if (iteration_ == 0)` branch (`chomp_optimizer.cpp:332-337`)
/// stores exactly this value as its first `best_group_trajectory_cost_`.
/// `best` is the last value that branch's `else if (cost <
/// best_group_trajectory_cost_)` (`chomp_optimizer.cpp:338`) accepted, and so
/// describes `best_group_trajectory_`, which is the trajectory
/// `optimize` copies out at `chomp_optimizer.cpp:507` and the one a caller
/// receives.
///
/// Keeping the three in one type rather than three `Option` fields is what
/// makes "seed measured but best not" unrepresentable: all three are observed
/// on the same passes or none is.
///
/// # Why `last` exists, and why a seed-vs-best claim alone would be vacuous
///
/// `best` starts at `seed` on iteration 0 and is replaced only when
/// `cost < best_group_trajectory_cost_` (`chomp_optimizer.cpp:338`), so
/// `best.total() <= seed.total()` **by construction**. A "did CHOMP improve
/// the objective?" measurement that compares those two can only ever answer
/// yes; its zero count of regressions would be a property of the min-tracking,
/// not of the optimizer.
///
/// The falsifiable quantity is `last`: the objective at the final iteration
/// the loop evaluated. Upstream computes it (it is `cost` on the last pass)
/// and discards it, and the whole reason upstream keeps a
/// `best_group_trajectory_` snapshot at all is that the iterate is *not*
/// monotone -- gradient descent at a fixed `learning_rate` can and does climb.
/// `seed` vs `last` is therefore the paired claim with an open sign, and
/// `last` vs `best` counts how often the snapshot is what saved the answer.
///
/// One honest limit on `last`: the loop evaluates the objective at the *top*
/// of each pass, before that pass's increments are applied
/// (`chomp_optimizer.cpp:305-308` precedes `addIncrementsToTrajectory`), so
/// `last` describes the iterate entering the final pass, not the trajectory
/// left after the final increment. Upstream never evaluates that one either.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ChompObjectiveProgress {
    /// The objective of the trajectory handed to the optimizer, evaluated
    /// before the first increment.
    pub seed: ChompObjective,
    /// The objective of the trajectory the optimizer returns.
    pub best: ChompObjective,
    /// The objective at the last iteration the loop evaluated -- which the
    /// optimizer discards in favour of `best`. May exceed `seed`.
    pub last: ChompObjective,
}

impl ChompObjectiveProgress {
    /// `seed.total() - best.total()`: how much better the *returned*
    /// trajectory is than the seed.
    ///
    /// Non-negative by construction (see this type's doc); a negative value
    /// would mean the port's accept rule had drifted from
    /// `chomp_optimizer.cpp:338`. Read it as a bound the type guarantees, not
    /// as a measurement of the optimizer.
    #[must_use]
    pub fn improvement(&self) -> f64 {
        self.seed.total() - self.best.total()
    }

    /// `seed.total() - last.total()`: how much the descent itself moved the
    /// objective, ignoring the best-snapshot.
    ///
    /// **Negative when CHOMP drove the objective above where it started.**
    /// This is the sign the seed-vs-`best` comparison cannot expose.
    #[must_use]
    pub fn descent(&self) -> f64 {
        self.seed.total() - self.last.total()
    }
}

/// Which of `optimize`'s three exits ended the loop.
///
/// Upstream has exactly these three and no more: the `for` condition, the
/// wall-clock `break`, and the `should_break_out` `break`. A fourth would be
/// a port deviation, which is why this is an enum rather than a pair of
/// booleans a caller has to interpret.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ChompExit {
    /// `iteration_ < max_iterations_` went false --
    /// `moveit_planners/chomp/chomp_motion_planner/src/chomp_optimizer.cpp:303`.
    IterationBound,
    /// The wall-clock `break` at
    /// `moveit_planners/chomp/chomp_motion_planner/src/chomp_optimizer.cpp:421-426`.
    ClockLimit,
    /// The `should_break_out` `break` at
    /// `moveit_planners/chomp/chomp_motion_planner/src/chomp_optimizer.cpp:477-487`.
    BreakOut,
}

/// What `optimize`'s loop did, as distinct from what its objective was.
///
/// [`ChompObjectiveProgress`] answers "did the returned trajectory cost less
/// than the seed". It cannot answer *why* the answer was no, because all four
/// of the candidate reasons -- a seed at a local minimum, a zero collision
/// term, an update computed and rejected, an update scaled below what the
/// cost resolves -- produce the same `improvement == 0`. They differ in how
/// many times the loop evaluated the objective at all, in whether the accept
/// branch ever fired, and in how large the applied update was; this type
/// carries those.
///
/// # Deviation
///
/// Upstream logs pieces of this and keeps the rest private:
/// `chomp_optimizer.cpp:370` logs `iteration_`, `chomp_optimizer.cpp:374`
/// logs the mesh-to-mesh break, `chomp_optimizer.cpp:417` logs the
/// over-threshold case, and `point_is_in_collision_`
/// (`chomp_optimizer.cpp:912`) is a member with no accessor. No caller of
/// `ChompPlanner::solve` can read any of it, so carrying it out is the same
/// class of deviation as [`ChompObjective`] and is recorded the same way, on
/// [`crate::planner::ChompSolution::loop_trace`].
#[derive(Debug, Clone, PartialEq)]
pub struct ChompLoopTrace {
    /// How many times the loop body ran, i.e. how many times the objective
    /// was evaluated. `1` means the loop left before any updated iterate was
    /// ever costed -- upstream evaluates at the top of the pass and applies
    /// its increments after
    /// (`moveit_planners/chomp/chomp_motion_planner/src/chomp_optimizer.cpp:305-354`).
    pub evaluations: u32,
    /// Which exit ran.
    pub exit: ChompExit,
    /// How many passes replaced `best` through the accept branch
    /// (`moveit_planners/chomp/chomp_motion_planner/src/chomp_optimizer.cpp:338`).
    /// The iteration-0 seeding is not counted: `0` means the seed was never
    /// beaten.
    pub accepted: u32,
    /// How many passes actually *ran* the mesh-to-mesh check at all --
    /// `iteration_ % 10 == 0`
    /// (`moveit_planners/chomp/chomp_motion_planner/src/chomp_optimizer.cpp:368`)
    /// -- regardless of what it found. [`Self::mesh_free_passes`] is a
    /// subset of this count, and the two must be read together:
    /// `mesh_free_passes == 0` with `mesh_checks > 0` means the check ran
    /// and never found the trajectory free; `mesh_checks == 0` means the
    /// check never ran (`max_iterations` too small to hit a multiple of 10,
    /// or the loop broke out before iteration 0's check), and
    /// `mesh_free_passes` alone cannot tell those two apart.
    pub mesh_checks: u32,
    /// How many passes found the mesh-to-mesh check collision free
    /// (`moveit_planners/chomp/chomp_motion_planner/src/chomp_optimizer.cpp:371`).
    /// A subset of [`Self::mesh_checks`]; see that field's doc for why
    /// `mesh_checks` has to accompany it.
    pub mesh_free_passes: u32,
    /// How many passes actually *ran* the collision-threshold comparison at
    /// all -- `!filter_mode_`
    /// (`moveit_planners/chomp/chomp_motion_planner/src/chomp_optimizer.cpp:406`)
    /// -- regardless of the comparison's outcome. [`Self::below_threshold_passes`]
    /// is a subset of this count, and the two must be read together:
    /// `below_threshold_passes == 0` with `threshold_checks > 0` means
    /// `c_cost` was compared every pass and never fell under
    /// `collision_threshold_`; `threshold_checks == 0` means `filter_mode`
    /// disabled the comparison entirely, and `below_threshold_passes` alone
    /// cannot tell those two apart.
    pub threshold_checks: u32,
    /// How many passes had `c_cost < collision_threshold_`
    /// (`moveit_planners/chomp/chomp_motion_planner/src/chomp_optimizer.cpp:408`).
    /// A subset of [`Self::threshold_checks`]; see that field's doc for why
    /// `threshold_checks` has to accompany it.
    pub below_threshold_passes: u32,
    /// Collision points inside their own clearance band on the seed --
    /// `get_potential` non-zero, i.e. the points the collision term is a
    /// function of at all.
    ///
    /// Counted over the **free segment** (`free_vars_start ..=
    /// free_vars_end`), because that is the range `get_collision_cost` sums
    /// over. The field below is counted over a *wider* range and against a
    /// *different* predicate, so the two are not comparable as a
    /// subset/superset pair -- see its own doc.
    pub seed_points_within_clearance: u32,
    /// Collision points upstream's own `point_is_in_collision_` predicate
    /// (`moveit_planners/chomp/chomp_motion_planner/src/chomp_optimizer.cpp:912`)
    /// flags on the seed.
    ///
    /// Counted over the **whole padded trajectory**, not the free segment:
    /// upstream's `performForwardKinematics` walks `0 ..= num_vars_all_ - 1`
    /// on `iteration_ == 0` and the free segment only afterwards
    /// (`moveit_planners/chomp/chomp_motion_planner/src/chomp_optimizer.cpp:868-874`),
    /// and this count is taken on that pass. The predicate differs too:
    /// `distance - radius < radius` here against `get_potential(..) > 0.0`
    /// above, which is `distance < radius + min_clearance`. Neither count
    /// bounds the other -- a sphere with `radius > min_clearance` can be
    /// flagged in collision while its potential is still `0.0`.
    pub seed_points_in_collision: u32,
    /// The largest absolute per-variable change the first pass actually
    /// applied, *after* `joint_update_limit`'s per-joint rescale
    /// (`moveit_planners/chomp/chomp_motion_planner/src/chomp_optimizer.cpp:643-661`).
    /// This is the update the next evaluation would have costed.
    pub first_pass_max_update: f64,
    /// [`ChompObjective::collision`] (upstream's `c_cost`,
    /// `moveit_planners/chomp/chomp_motion_planner/src/chomp_optimizer.cpp:306`)
    /// at every evaluated pass, in pass order. `collision_costs.len() ==
    /// evaluations as usize` always. Upstream computes this same value every
    /// pass and only ever lets it out at `RCLCPP_DEBUG` (`:310`); this is
    /// that log line, structured, for the same reason
    /// [`ChompObjective`] carries the value out at all -- no caller of
    /// `ChompPlanner::solve` can otherwise tell "still descending" apart
    /// from "plateaued above `collision_threshold_`" without re-running the
    /// optimizer under a debug logger.
    pub collision_costs: Vec<f64>,
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
    /// [`cspace_collision::distance_field::DistanceFieldCollisionCache::get_collision_gradients`]
    /// itself requires.
    pub env_distance_field: &'a dyn DistanceField,
}

/// Builds, once per [`ChompOptimizer::new`] call, the reverse joint/link
/// lookups upstream reads directly off `JointModel::getParentLinkModel`/
/// `getChildLinkModel` -- neither is exposed by [`cspace_core::model::joint::JointModel`]
/// (it only carries a `parent_link_index` internally), so this port derives
/// both by scanning [`RobotModel::link_models`] once: a link's
/// [`cspace_core::model::LinkModel::parent_joint_index`] is that joint's *child*
/// link, and every entry of the link's own
/// [`cspace_core::model::LinkModel::child_joint_indices`] has that link as its
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
/// ([`cspace_collision::distance_field::DistanceFieldCollisionCache::get_collision_gradients`]'s
/// always-fresh-build) -- gating on it would have returned a zero-length
/// vector regardless of the real per-link sphere counts. `cspace-distance-field`
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
///   established in `cspace-distance-field` (`env_distance_field: &dyn
///   DistanceField` is threaded the same way through
///   [`cspace_collision::distance_field::DistanceFieldCollisionCache::get_collision_gradients`]
///   itself) -- this removes lifetime-parameter proliferation from the
///   struct (only `'m`, the robot model's lifetime, remains) at the cost
///   of a few more call-site arguments.
/// - **`gsr_`/`GroupStateRepresentation` is never stored.** Its accessor,
///   [`cspace_collision::distance_field::DistanceFieldCollisionCache::get_collision_gradients`],
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
///   wiring this as a method today would make `cspace-planners-chomp` --
///   per round 20's brief, and the `hy_env_`/`getCollisionGradients`
///   evidence backing it -- depend on two crates it has never carried:
///   `cspace-scene` (for `PlanningScene::is_path_valid`,
///   `scene.rs:1725`) and `cspace-collision` (for `ParryCollisionEnv`,
///   `parry.rs:1629` -- the only existing implementer of the
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
///   `cspace_planning::scene::PlanningScene::is_path_valid` is ported and generic over
///   `E: CollisionEnv<Posed>`, and `cspace_collision::ParryCollisionEnv`
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
///   typed as [`cspace_collision::distance_field::DistanceFieldCollisionCache`]; Rust
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
///   `axis = joint_transform * axis;` (`chomp_optimizer.cpp:749`) is ported
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
///   `cspace_core::sampling::MultivariateGaussian` in this crate; `rsl` itself is
///   not ported (D1: not a numeric-core dependency).
/// - **`calculateCollisionIncrements`'s two independent `should_break_out`
///   conditions in `optimize` (the `iteration_ % 10 == 0` mesh-to-mesh
///   check and the `!filter_mode_` collision-threshold check) are kept as
///   two separate, unconditionally-evaluated `if` blocks, not collapsed
///   into `if / else if`.** Both can still fire in the same pass
///   (`chomp_optimizer.cpp:367-410`, reachable: `iteration % 10 == 0` and
///   `c_cost < collision_threshold` are independent conditions, both true
///   on pass 0 with an empty env field). Upstream writes both
///   `num_collision_free_iterations_` unconditionally, so whichever block
///   runs second (always the threshold block, textually) silently
///   overwrites the mesh check's `0` with `max_iterations_after_collision_free_`,
///   discarding a ground-truth mesh-safety confirmation
///   (`isCurrentTrajectoryMeshToMeshCollisionFree`, `:520-537`) in favor of
///   a sphere/distance-field cost proxy's weaker one
///   (`getCollisionCost`, `:691-`). Fixed here, not reproduced: the
///   threshold block's write to `num_collision_free_iterations` is now
///   conditional on the mesh check not having already confirmed safety
///   this same pass, so the stronger signal wins regardless of which block
///   runs first (pinned by
///   `optimize_collision_threshold_no_longer_discards_mesh_to_meshs_immediate_break_signal`).
///   These same two `if` blocks used to also each call
///   `iteration_++`/`self.iteration += 1` independently, which could
///   double- or triple-advance the pass counter in one loop pass -- that
///   part is also fixed, not reproduced; see `optimize`'s own doc comment.
///   Each block's own outer gate (`iteration_ % 10 == 0`, `!filter_mode_`)
///   now also increments a `mesh_checks`/`threshold_checks` counter
///   independent of whether the inner condition fires, so
///   [`ChompLoopTrace`] can distinguish "checked and never found free/below
///   threshold" from "never checked at all" -- see
///   [`ChompLoopTrace::mesh_checks`]/[`ChompLoopTrace::threshold_checks`].
/// - **Dead/write-only upstream fields are not ported at all**, verified
///   via `rg` across the whole `chomp_motion_planner` package, not just
///   `chomp_optimizer.cpp`: `group_trajectory_backup_` (read only inside
///   fully-commented-out HMC-perturbation code), `state_is_in_collision_`
///   (written every `performForwardKinematics` call, never read anywhere),
///   the *stored* `point_is_in_collision_` 2D field (it has two reads: a
///   live one immediately after assignment, in the same statement block
///   that also sets `state_is_in_collision_`/`is_collision_free_`
///   (`chomp_optimizer.cpp:914`) -- this is the computation that survives
///   as an inline local `bool` in
///   [`ChompOptimizer::perform_forward_kinematics`], just not as a
///   persisted field -- and a separate, genuinely dead read inside a
///   `/* */`-commented block (`:615`)), and the entire dead-HMC-path field
///   set already
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
    /// Upstream's `best_group_trajectory_cost_` (`chomp_optimizer.hpp:150`),
    /// kept decomposed and optional rather than as that member's `double`.
    ///
    /// Upstream initialises it to nothing in particular and relies on
    /// `iteration_ == 0` running first to seed it; with `max_iterations_ == 0`
    /// the loop body never executes and the member is read only as whatever it
    /// was. `None` here is that same state made unmistakable, so a caller
    /// cannot read an unmeasured objective as a real `0.0`. The
    /// `iteration_ == 0` arm of `chomp_optimizer.cpp:332-338` is exactly
    /// `is_none()`, which is why [`ChompOptimizer::optimize`] resets it.
    best_objective: Option<ChompObjectiveProgress>,
    /// What the last [`ChompOptimizer::optimize`] call's loop did. `None`
    /// before the first call, on exactly the same condition as
    /// `best_objective`.
    loop_trace: Option<ChompLoopTrace>,
    last_improvement_iteration: i32,
    num_collision_free_iterations: u32,

    is_collision_free: bool,
    /// Upstream keeps `point_is_in_collision_` per point
    /// (`moveit_planners/chomp/chomp_motion_planner/src/chomp_optimizer.cpp:912`)
    /// and reads it in one place this port already covers; only the count is
    /// kept here, refreshed by every `perform_forward_kinematics`.
    points_in_collision: u32,
    worst_collision_cost_state: i64,
}

impl<'m> ChompOptimizer<'m> {
    /// Ported from `ChompOptimizer::ChompOptimizer` (`chomp_optimizer.cpp:61-85`) plus
    /// `initialize` (`chomp_optimizer.cpp:87-244`). Unlike upstream, initialization
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
            cost.scale(max_cost_scale)?;
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
            best_objective: None,
            loop_trace: None,
            last_improvement_iteration: -1,
            num_collision_free_iterations: 0,
            is_collision_free: false,
            points_in_collision: 0,
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
    /// [`cspace_core::state::RobotState::set_joint_positions`] rather than
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

    /// Ported from `performForwardKinematics` (`chomp_optimizer.cpp:862-944`).
    ///
    /// # `sphere_locations` gap closed (round 26)
    ///
    /// Rounds 20 through 25 read collision-point positions from
    /// `GroupStateRepresentation::link_body_decompositions[link_index].sphere_centers()`
    /// instead of `GradientInfo::sphere_locations`, because
    /// `DistanceFieldCollisionCache::get_collision_gradients` always takes
    /// the fresh-build path (no long-lived `gsr_` to reuse the way
    /// `chomp_optimizer.cpp` does), and that path left `sphere_locations`
    /// empty for every link. `cspace-distance-field` round 25 (`f5328da`)
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
        self.points_in_collision = 0;

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
                        self.points_in_collision += 1;
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
                // Division/NaN-guard audit (round: chomp/stomp sweep): `idx`'s
                // `as usize` cannot wrap a negative value here. `offset` is
                // bounded to `-(DIFF_RULE_LENGTH / 2)..=(DIFF_RULE_LENGTH / 2)`
                // (`k` ranges over `DIFF_RULES`'s own `DIFF_RULE_LENGTH`-wide
                // rows), and `i >= self.free_vars_start`, which is always
                // exactly `DIFF_RULE_LENGTH - 1` -- `ChompOptimizer::new` is
                // this type's only constructor, and it always derives
                // `free_vars_start` from `ChompTrajectory::from_source_trajectory`'s
                // own `start_index = diff_rule_length - 1`, called with this
                // crate's fixed `DIFF_RULE_LENGTH` constant, never a
                // caller-supplied value (the caller-facing
                // `set_start_end_index` cannot reach this: `new` builds its
                // own `group_trajectory` from `full_trajectory`'s data, not
                // its start/end index). So `i + offset >= (DIFF_RULE_LENGTH -
                // 1) - (DIFF_RULE_LENGTH / 2)`, `3` for this crate's
                // `DIFF_RULE_LENGTH == 7`, always `>= 0`. (If `free_vars_end <
                // free_vars_start`, the outer loop range is empty and this
                // line never runs at all.)
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
    ///
    /// # `raw as i64` cannot see a NaN/infinite `raw` (division/NaN-guard
    /// audit, round: chomp/stomp sweep)
    ///
    /// `raw`'s only inputs are `rng.random_range(0.0..1.0)` and
    /// `free_vars_start`/`free_vars_end` widened to `f64`. `random_range`'s
    /// own implementation (`rand-0.10.2/src/distr/uniform_float.rs`,
    /// `UniformSampler::new`/`sample_single_inclusive`) rejects a non-finite
    /// bound before sampling and guarantees the result lands in `[low,
    /// high)` -- with the literal bounds `0.0..1.0` here, always finite.
    /// `free_vars_start as f64`/`free_vars_end as f64` are exact,
    /// non-negative, finite widenings of a `usize` too small to lose
    /// precision at `f64`'s 53-bit mantissa (a trajectory large enough to
    /// overflow that would already have failed an earlier `DMatrix`
    /// allocation). So every term of `raw`'s sum is finite, and finite +
    /// finite * finite is finite: `raw as i64` never sees `NaN` or an
    /// infinite operand, so the saturating float-to-int cast (Rust 1.45+)
    /// never actually saturates here -- it is a plain truncation of an
    /// in-range value. The two `if` clamps immediately below additionally
    /// re-bound whatever it truncates to into `[free_vars_start,
    /// free_vars_end]` regardless, so nothing downstream depends on this
    /// unreachability holding. No fix applies: an `is_finite()` check
    /// against an input that cannot occur would be exactly the defensive
    /// validation this port's own conventions reject.
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
                let vel = self.collision_point_vel_eigen[i][j];
                let vel_mag = self.collision_point_vel_mag[i][j];
                if vel_mag == 0.0 {
                    // Upstream's `normalized_velocity = ... / vel_mag`
                    // (`chomp_optimizer.cpp:596`) is unguarded and produces
                    // `NaN` here (`0.0 / 0.0`); this port's own swept-cost
                    // semantic (see `get_collision_cost`'s doc: a
                    // stationary trajectory contributes exactly `0.0`
                    // regardless of penetration depth) says a zero-velocity
                    // point must contribute exactly `0.0`, which
                    // `cartesian_gradient`'s `vel_mag * (...)` factor
                    // already implies for any *finite* `(...)` -- skipping
                    // here makes that hold even when `(...)` would
                    // otherwise be `NaN`, matching `vel_mag * finite ==
                    // 0.0` exactly instead of `0.0 * NaN == NaN`.
                    continue;
                }
                let potential_gradient = -self.collision_point_potential_gradient[i][j];
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
    ///
    /// This is not a gap upstream closes inside `chomp_planner.cpp`
    /// either: `ChompPlanner::solve` (`chomp_planner.cpp:80`) only calls
    /// `start_state.satisfiesBounds()` before planning -- no
    /// `planning_scene->isStateColliding(start_state, ...)` or equivalent.
    /// Upstream's actual guard against a stationary in-collision start
    /// (or any other silently-wrong-but-`SUCCESS`-flagged response) lives
    /// one layer up, in `move_group`'s planning pipeline:
    /// `default_planning_response_adapters::ValidateSolution`
    /// (`moveit_ros/planning/planning_response_adapter_plugins/src/validate_path.cpp`)
    /// re-checks *every* waypoint of the returned trajectory -- including
    /// waypoint 0, the start state -- via `planning_scene->isPathValid`,
    /// a real `checkCollision` call independent of CHOMP's own
    /// velocity-weighted internal metric, and downgrades the response to
    /// `INVALID_MOTION_PLAN` if any waypoint fails. This port already has
    /// that adapter: `cspace_planning::response_adapters::ValidateSolution`
    /// (`crates/cspace-planning/src/response_adapters/validate_path.rs`;
    /// not a dependency of this crate, hence a plain path here, not a
    /// doc-link). The gap is composition, not a missing component:
    /// [`crate::solve`]
    /// is a bespoke function outside `cspace-planning`'s adapter pipeline
    /// (see this crate's `# Deviation: no cspace-scene, no cspace-planning
    /// dependency` note), so nothing currently routes its response through
    /// `ValidateSolution` before a caller treats it as accepted -- matching
    /// upstream's own architecture, where `chomp_planner.cpp` alone has
    /// this exact same gap and only closes it when composed inside
    /// `move_group`'s pipeline. A future dispatcher wiring
    /// `cspace-planners-chomp::solve`'s output through
    /// `cspace-planning::response_adapters::ValidateSolution` (the same
    /// missing dispatcher [`crate::planner::ChompGoal`]'s doc comment
    /// already names) is
    /// where this should be closed, not a change to this function.
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
                // Division/NaN-guard audit (round: chomp/stomp sweep):
                // `usize -> i64` widening, distinct from this file's other
                // audited `f64 -> i*`/`i64 -> usize` casts -- there is no
                // fractional or negative source value to lose. It can only
                // misrepresent `i` if `i > i64::MAX`, which would require a
                // trajectory point count `DMatrix` allocation elsewhere in
                // this type already could not have made.
                self.worst_collision_cost_state = i as i64;
            }
        }
        self.parameters.obstacle_cost_weight * collision_cost
    }

    /// Collision points whose potential is non-zero over the free segment --
    /// the points `get_collision_cost` sums anything but `0.0` for. A point
    /// outside its clearance band contributes nothing to the collision term
    /// and nothing to the collision gradient (`get_potential` returns `0.0`
    /// there), so this is the size of the support of the collision half of
    /// the objective.
    fn points_within_clearance(&self) -> u32 {
        let mut n = 0;
        for i in self.free_vars_start..=self.free_vars_end {
            for j in 0..self.num_collision_points {
                if self.collision_point_potential[i][j] > 0.0 {
                    n += 1;
                }
            }
        }
        n
    }

    /// What the last [`ChompOptimizer::optimize`] call's loop did, or `None`
    /// if it has not been called.
    ///
    /// No upstream counterpart, for the reason [`ChompLoopTrace`]'s doc
    /// gives.
    #[must_use]
    pub fn loop_trace(&self) -> Option<ChompLoopTrace> {
        self.loop_trace.clone()
    }

    /// The objective at the seed trajectory and at the trajectory
    /// [`ChompOptimizer::optimize`] returned, or `None` if no iteration ever
    /// ran (`max_iterations == 0`).
    ///
    /// No upstream counterpart: `best_group_trajectory_cost_` is private
    /// (`chomp_optimizer.hpp:150`) and `optimize` returns `bool`. See
    /// [`ChompObjective`]'s doc for the one upstream line that does emit these
    /// two numbers, and why they are reported separately here.
    #[must_use]
    pub fn objective(&self) -> Option<ChompObjectiveProgress> {
        self.best_objective
    }

    /// Ported from `getTrajectoryCost`.
    pub fn get_trajectory_cost(&mut self) -> Result<f64> {
        Ok(get_smoothness_cost(
            &self.joint_costs,
            &self.group_trajectory,
            self.parameters.smoothness_cost_weight,
        )? + self.get_collision_cost())
    }

    /// Ported from `ChompOptimizer::optimize` (`chomp_optimizer.cpp:290-518`).
    /// See this type's doc comment for the closure `mesh_to_mesh_collision_free`
    /// replaces, and for why its two `should_break_out` conditions are kept
    /// as two independent `if` blocks rather than collapsed.
    ///
    /// # Deviation: `self.iteration` advances by exactly one pass, never two
    ///
    /// Upstream's `for (iteration_ = 0; iteration_ < max_iterations_;
    /// ++iteration_)` (`chomp_optimizer.cpp:303`) increments `iteration_`
    /// unconditionally at the end of every pass, *on top of* a branch-local
    /// `iteration_++;` at `:376` (the mesh check) and another at `:412` (the
    /// collision-threshold check) that each also fire when their own
    /// condition is met. Upstream's `should_break_out` gate (`:477-486`)
    /// does not `break` on every pass where `should_break_out` was set --
    /// only once the grace period (`num_collision_free_iterations_`, from
    /// `parameters_->max_iterations_after_collision_free_`) is exhausted --
    /// so a pass that trips the collision-threshold branch (the common
    /// case: its grace period defaults to `5`, not `0`) falls through to
    /// the `for` loop's own `++iteration_` on top of its own branch-local
    /// one, advancing `iteration_` by 2 for that single executed pass; both
    /// branches firing on the same pass advances it by 3. Reachable on
    /// ordinary inputs, not a corner case requiring a crafted fixture.
    ///
    /// This port no longer reproduces it. It was `doc/upstream-bugs.md`'s
    /// `chomp-iteration-double-increment` (`reproduced-grandfathered`
    /// pending a fresh decision, per that entry's own text) before that
    /// file was deleted; `GOALS.md` records its own table's Phase conditions
    /// met as of 2026-08-09 (Phase 8's other baseline, C++ OMPL RRTConnect,
    /// is qualified there and is not what this sentence rests on), moving
    /// this project from "transcribe the numerics" to fixing code defects
    /// against upstream, and this is one such fix. The two branch-local
    /// increments below are gone; the unconditional increment at this
    /// loop's end is now `self.iteration`'s only writer,
    /// so every executed pass advances it by exactly one step, matching how
    /// [`ChompLoopTrace::evaluations`] (which was never affected by this
    /// bug) already counts passes. This does not change whether the loop
    /// terminates or what it returns -- `is_collision_free`/
    /// `best_group_trajectory` never read `self.iteration`'s absolute
    /// value -- only: (a) how many optimization passes run before
    /// `max_iterations` is exhausted, since a grace-period pass no longer
    /// consumes 2-3 units of that budget for one executed pass; (b) which
    /// iteration index the every-10th-pass mesh recheck
    /// (`self.iteration % 10 == 0`) lands on during a grace period, since
    /// it no longer drifts off a clean multiple of 10.
    pub fn optimize(
        &mut self,
        full_trajectory: &mut ChompTrajectory,
        collision: &mut ChompCollisionContext<'_, 'm>,
        mesh_to_mesh_collision_free: &mut dyn FnMut(&RobotState<'m>, &DMatrix<f64>) -> bool,
        rng: &mut impl Rng,
    ) -> Result<bool> {
        let start_time = Instant::now();

        self.iteration = 0;
        // Upstream's `iteration_ == 0` arm re-seeds `best_group_trajectory_cost_`
        // on every entry to `optimize`, so a second call on the same optimizer
        // does not inherit the first call's best. `best_objective.is_none()`
        // stands in for that test below, so it has to be cleared here for the
        // two to stay the same condition.
        self.best_objective = None;
        // Same reset, same reason: a trace that survived into a second call
        // would describe the first call's loop.
        let mut evaluations: u32 = 0;
        let mut accepted: u32 = 0;
        let mut mesh_checks: u32 = 0;
        let mut mesh_free_passes: u32 = 0;
        let mut threshold_checks: u32 = 0;
        let mut below_threshold_passes: u32 = 0;
        let mut seed_points_within_clearance: u32 = 0;
        let mut seed_points_in_collision: u32 = 0;
        let mut first_pass_max_update = 0.0f64;
        let mut collision_costs: Vec<f64> = Vec::new();
        let mut exit = ChompExit::IterationBound;
        // Declared once, outside the loop, matching upstream
        // (`chomp_optimizer.cpp:300`): once any pass sets it, it stays
        // `true` for every later pass in this call, so
        // `collision_free_iteration` below keeps incrementing on later
        // passes even when that pass's own mesh/threshold condition
        // doesn't refire. Upstream never resets it inside the loop -- a
        // per-pass-local `let mut should_break_out = false;` here would
        // silently narrow that persisted-until-break semantics to
        // "refires every single pass", which is a different condition.
        let mut should_break_out = false;
        while self.iteration < self.parameters.max_iterations {
            self.perform_forward_kinematics(collision)?;
            evaluations += 1;
            let c_cost = self.get_collision_cost();
            collision_costs.push(c_cost);
            let s_cost = get_smoothness_cost(
                &self.joint_costs,
                &self.group_trajectory,
                self.parameters.smoothness_cost_weight,
            )?;
            let objective = ChompObjective {
                smoothness: s_cost,
                collision: c_cost,
            };
            let cost = objective.total();

            // `chomp_optimizer.cpp:332-341`. `is_none()` is upstream's
            // `iteration_ == 0` (both are true on exactly the pass that has no
            // previous best), and `seed` is fixed on that same pass because
            // upstream never revisits its iteration-0 assignment.
            match self.best_objective {
                None => {
                    self.best_group_trajectory = self.group_trajectory.trajectory_matrix().clone();
                    self.best_objective = Some(ChompObjectiveProgress {
                        seed: objective,
                        best: objective,
                        last: objective,
                    });
                    self.last_improvement_iteration = self.iteration;
                    seed_points_in_collision = self.points_in_collision;
                    seed_points_within_clearance = self.points_within_clearance();
                }
                Some(ref mut progress) => {
                    // `last` tracks every evaluated pass, not just accepted
                    // ones -- that is the whole point of it (see
                    // `ChompObjectiveProgress`'s doc).
                    progress.last = objective;
                    if cost < progress.best.total() {
                        self.best_group_trajectory =
                            self.group_trajectory.trajectory_matrix().clone();
                        progress.best = objective;
                        self.last_improvement_iteration = self.iteration;
                        accepted += 1;
                    }
                }
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
            let applied_max = add_increments_to_trajectory(
                &mut self.group_trajectory,
                &final_increments,
                self.parameters.joint_update_limit,
            )?;
            if evaluations == 1 {
                first_pass_max_update = applied_max;
            }

            handle_joint_limits(
                self.robot_model,
                self.joint_model_group,
                &mut self.group_trajectory,
                &self.joint_costs,
            )?;
            full_trajectory.update_from_group_trajectory(&self.group_trajectory);

            // `mesh_confirmed_this_pass` is declared outside the `iteration_ %
            // 10 == 0` gate (not folded into one `&&` expression) so the
            // threshold block below can read it regardless of whether this
            // pass was even a check pass, and so `mesh_checks` can count the
            // gate firing independent of what the inner check finds.
            let mut mesh_confirmed_this_pass = false;
            if self.iteration % 10 == 0 {
                mesh_checks += 1;
                if mesh_to_mesh_collision_free(&self.start_state, &self.best_group_trajectory) {
                    mesh_confirmed_this_pass = true;
                    self.num_collision_free_iterations = 0;
                    self.is_collision_free = true;
                    should_break_out = true;
                    mesh_free_passes += 1;
                }
            }

            if !self.parameters.filter_mode {
                threshold_checks += 1;
                if c_cost < self.parameters.collision_threshold {
                    self.is_collision_free = true;
                    should_break_out = true;
                    below_threshold_passes += 1;
                    // Deviation: `num_collision_free_iterations` is written here only
                    // if the mesh-to-mesh check above did not already confirm safety
                    // this same pass. Upstream (`chomp_optimizer.cpp:373`/`:410`)
                    // writes both unconditionally, so whichever block runs second
                    // wins when both fire in one pass -- reachable, since
                    // `iteration % 10 == 0` and `c_cost < collision_threshold` are
                    // independent conditions, trivially both true on pass 0 with an
                    // empty env field. Textually the threshold block runs second, so
                    // it always overwrites the mesh block's `0` with the (larger)
                    // grace period, discarding the ground-truth mesh check's
                    // "already verified, break at the next check" signal in favor of
                    // its own weaker one. `isCurrentTrajectoryMeshToMeshCollisionFree`
                    // (`chomp_optimizer.cpp:520-537`) directly validates the actual
                    // trajectory via `planning_scene_->isPathValid`; `c_cost`
                    // (`getCollisionCost`, `:691-`) is a sphere/distance-field cost
                    // sum, a proxy for the same thing computed without ever calling
                    // the mesh check. The mesh signal must win, and win regardless of
                    // which block happens to run first -- not by reordering the two
                    // (still fragile to a future reorder), but by making the write
                    // explicitly conditional on the stronger signal not already
                    // having fired.
                    if !mesh_confirmed_this_pass {
                        self.num_collision_free_iterations =
                            self.parameters.max_iterations_after_collision_free as u32;
                    }
                }
            }

            if start_time.elapsed().as_secs_f64() > self.parameters.planning_time_limit {
                exit = ChompExit::ClockLimit;
                break;
            }

            if should_break_out {
                self.collision_free_iteration += 1;
                if self.num_collision_free_iterations == 0 {
                    exit = ChompExit::BreakOut;
                    break;
                } else if self.collision_free_iteration > self.num_collision_free_iterations {
                    // Upstream's own check for exactly this moment, commented out and
                    // never run (`chomp_optimizer.cpp:486-490`: a `checkCurrentIterValidity()`
                    // re-check guarding a `ROS_WARN("Apparently regressed")`). The grace
                    // period just expired without ever re-confirming mesh safety on
                    // whatever `best_group_trajectory` landed on -- accepts are driven by
                    // *total* cost (smoothness + collision, the `cost < progress.best.total()`
                    // arm above), not mesh safety, so a pass during the grace window can
                    // replace it with something the mesh check below would reject. A
                    // printed warning does not stop the caller from receiving a
                    // trajectory it was told is collision-free when it is not, so this
                    // re-runs the same predicate the mesh branch above already trusts, on
                    // the trajectory that is about to be returned, and corrects
                    // `is_collision_free` instead of merely noting the discrepancy. The
                    // `num_collision_free_iterations == 0` arm above needs no equivalent
                    // check: it only ever fires on the same pass the mesh branch already
                    // verified `best_group_trajectory` directly, with no write to it in
                    // between.
                    if !mesh_to_mesh_collision_free(&self.start_state, &self.best_group_trajectory)
                    {
                        self.is_collision_free = false;
                    }
                    exit = ChompExit::BreakOut;
                    break;
                }
            }

            self.iteration += 1;
        }

        self.loop_trace = Some(ChompLoopTrace {
            evaluations,
            exit,
            accepted,
            mesh_checks,
            mesh_free_passes,
            threshold_checks,
            below_threshold_passes,
            seed_points_within_clearance,
            seed_points_in_collision,
            first_pass_max_update,
            collision_costs,
        });

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
    use cspace_core::model::MeshSearchPaths;
    use cspace_core::srdf::SrdfModel;
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

    use cspace_collision::LinkPaddingScale;
    use cspace_collision::distance_field::{
        DistanceFieldConfig, DistanceGradient, GridGeometry, PropagationDistanceField,
        add_link_body_decompositions,
    };
    use rand::SeedableRng;
    use rand_chacha::ChaCha8Rng;

    const CHOMP_COLLISION_GROUP: &str = "chain";

    /// A synthetic two-joint revolute chain with primitive (`<box>`)
    /// collision geometry, matching the construction idiom
    /// `cspace-distance-field`'s own `get_collision_gradients` tests use
    /// (`two_link_model_and_srdf` in
    /// `collision_env_distance_field.rs`). `panda.urdf`'s `<collision>`
    /// tags are all `<mesh>` references, which `MeshSearchPaths::none()`
    /// (this crate's own test setup) skips entirely per
    /// `cspace_core::model::MeshSearchPaths::none`'s own doc comment -- so
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
        cspace_core::test_support::assert_group_has_updated_links(&model, CHOMP_COLLISION_GROUP);
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
    /// rather than working around: as of `cspace-distance-field` round 25
    /// (`f5328da`), every [`GradientInfo`] this crate's only access path
    /// (`DistanceFieldCollisionCache::get_collision_gradients`) returns has
    /// `sphere_locations` populated, and it is element-for-element identical
    /// to `link_body_decompositions[i].sphere_centers()` -- the exact value
    /// the pre-round-26 workaround read instead. If `cspace-distance-field`
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

    /// Division/NaN-guard audit (round: chomp/stomp sweep). Upstream's
    /// `calculateCollisionIncrements` (`chomp_optimizer.cpp:596`) divides
    /// `collision_point_vel_eigen_[i][j]` by `vel_mag` with no zero guard --
    /// this port transcribes the same `vel / vel_mag` unguarded. `vel_mag ==
    /// 0.0` is not a synthetic corner case: it is exactly what every
    /// collision point on a fully stationary trajectory has (a `solve()`
    /// call whose `start_state` already satisfies `goal_constraints` builds
    /// one -- `build_seed_trajectory` never checks start != goal), and
    /// nothing stops that stationary pose from also being in collision
    /// (`build_seed_trajectory` checks joint bounds, never collision). This
    /// fixture constructs that exact state directly (an obstacle placed on a
    /// real collision sphere, as `perform_forward_kinematics_flags_the_
    /// point_an_obstacle_sits_on` above does, plus an explicit zero velocity
    /// -- engineering the precise boundary rather than fighting
    /// floating-point cancellation to hit it via a genuinely all-zero
    /// trajectory) rather than inventing a scenario no real caller reaches.
    ///
    /// The oracle is this file's own documented semantic, not a guess:
    /// [`ChompOptimizer::get_collision_cost`]'s doc says collision cost is
    /// swept, so "a perfectly stationary trajectory... returns exactly `0.0`
    /// here regardless of how deeply it penetrates an obstacle." The same
    /// swept-by-`vel_mag` structure is what `cartesian_gradient = vel_mag *
    /// (...)` computes for the *increment*: a zero-velocity point must
    /// contribute exactly zero to `collision_increments`, matching that cost
    /// semantic, not `NaN` from `0.0 / 0.0`.
    #[test]
    fn calculate_collision_increments_zero_velocity_point_contributes_nothing() {
        let model = chomp_collision_model();
        let source = chomp_full_trajectory(&model, 10);
        let start_state = RobotState::new(&model);
        let parameters = ChompParameters {
            // Deterministic: visit every free point, not one random one.
            use_stochastic_descent: false,
            ..ChompParameters::default()
        };

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
        let i = optimizer.free_vars_start;
        assert!(
            optimizer.collision_point_potential[i][0] > 0.0001,
            "test setup must land this collision point past calculate_collision_increments's \
             own 0.0001 potential threshold, or the division under test is never reached"
        );

        // The exact boundary: real potential (in collision), engineered
        // exactly-zero swept velocity.
        optimizer.collision_point_vel_eigen[i][0] = Vector3::zeros();
        optimizer.collision_point_vel_mag[i][0] = 0.0;

        let mut rng = ChaCha8Rng::seed_from_u64(1);
        let increments = optimizer.calculate_collision_increments(&mut rng).unwrap();

        assert!(
            increments.iter().all(|v| *v == 0.0),
            "a collision point with exactly zero swept velocity must contribute exactly 0.0 \
             (this file's own swept-cost semantic), not NaN from an unguarded 0.0/0.0: {increments}"
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

    /// `objective()` is `None` -- not `Some(0.0)` -- when the loop body never
    /// ran, which is the state upstream leaves `best_group_trajectory_cost_`
    /// in for `max_iterations_ == 0`.
    #[test]
    fn objective_is_none_when_no_iteration_ever_evaluated_it() {
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
            max_iterations: 0,
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
            optimizer.objective(),
            None,
            "zero iterations evaluated the objective zero times, so there is no value to report"
        );
    }

    /// A trajectory whose objective is identically zero cannot tell any of
    /// `seed`/`best`/`last` apart, and the all-zero `chomp_full_trajectory`
    /// fixture is exactly that. Every fixture below therefore drives the last
    /// point to a non-zero goal and fills the interior in, and this helper is
    /// the single place that happens.
    fn seeded_trajectory(model: &RobotModel, num_points: usize, goal: f64) -> ChompTrajectory {
        let mut source = chomp_full_trajectory(model, num_points);
        let last = source.num_points() - 1;
        let num_joints = source.num_joints();
        source.set_trajectory_point(last, &vec![goal; num_joints]);
        source.fill_in_min_jerk();
        source
    }

    /// Runs `optimize` on `seeded_trajectory(.., goal)` and returns the
    /// objective triple.
    fn objective_after_optimize(goal: f64, max_iterations: i32) -> ChompObjectiveProgress {
        let model = chomp_collision_model();
        let start_state = RobotState::new(&model);
        let source = seeded_trajectory(&model, 10, goal);
        let mut full = source.clone();
        let mut cache = chomp_collision_cache(&model);
        let field = env_field_with_points(&[]);
        let mut collision = ChompCollisionContext {
            cache: &mut cache,
            env_distance_field: &field,
        };
        let parameters = ChompParameters {
            max_iterations,
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
        optimizer.objective().expect("iterations ran")
    }

    /// `optimize`'s `best_objective = None` reset, which
    /// `objective_is_none_when_no_iteration_ever_evaluated_it` cannot see
    /// because it calls `optimize` once.
    ///
    /// Upstream re-seeds `best_group_trajectory_cost_` on every entry to
    /// `optimize` through its `iteration_ == 0` arm
    /// (`chomp_optimizer.cpp:332-337`); the port's `is_none()` stands in for
    /// that test, so without the reset a second call would keep the first
    /// call's `seed` and report an improvement it did not make. The second
    /// call starts from the trajectory the first one returned, so its `seed`
    /// is the first call's `best` -- an equality the reset is the only thing
    /// producing.
    #[test]
    fn a_second_optimize_reseeds_the_objective_from_the_first_calls_result() {
        let model = chomp_collision_model();
        let start_state = RobotState::new(&model);
        let source = seeded_trajectory(&model, 10, 1.0);
        let mut full = source.clone();
        let mut cache = chomp_collision_cache(&model);
        let field = env_field_with_points(&[]);
        let mut collision = ChompCollisionContext {
            cache: &mut cache,
            env_distance_field: &field,
        };
        let parameters = ChompParameters {
            max_iterations: 10,
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
        let first = optimizer.objective().expect("iterations ran");

        let mut rng = ChaCha8Rng::seed_from_u64(1);
        optimizer
            .optimize(&mut full, &mut collision, &mut |_, _| false, &mut rng)
            .unwrap();
        let second = optimizer.objective().expect("iterations ran");

        assert_relative_eq!(second.seed.total(), first.best.total(), epsilon = EPS);
        // Without the reset the second call would report the first call's
        // seed, so this is the inequality that makes the assertion above a
        // statement about the reset rather than an identity.
        assert!(
            (first.seed.total() - first.best.total()).abs() > EPS,
            "fixture no longer moves `best` off `seed`, so the reset cannot be observed"
        );
    }

    /// `seed` is iteration 0's objective, computed here on an optimizer that
    /// has taken no step so the assertion is not the loop checked against
    /// itself.
    #[test]
    fn objective_seed_is_the_untouched_trajectorys_two_cost_terms() {
        let model = chomp_collision_model();
        let start_state = RobotState::new(&model);
        let source = seeded_trajectory(&model, 10, 1.0);
        let mut cache = chomp_collision_cache(&model);
        let field = env_field_with_points(&[]);
        let mut collision = ChompCollisionContext {
            cache: &mut cache,
            env_distance_field: &field,
        };
        let parameters = ChompParameters {
            max_iterations: 5,
            filter_mode: true,
            ..ChompParameters::default()
        };

        let mut probe = ChompOptimizer::new(
            &source,
            CHOMP_COLLISION_GROUP,
            &parameters,
            &start_state,
            &mut collision,
            None,
        )
        .unwrap();
        probe.perform_forward_kinematics(&mut collision).unwrap();
        let expected_collision = probe.get_collision_cost();
        let expected_smoothness = get_smoothness_cost(
            &probe.joint_costs,
            &probe.group_trajectory,
            parameters.smoothness_cost_weight,
        )
        .unwrap();
        drop(probe);

        // A zero objective would make every assertion below vacuous -- this
        // fixture has to have a real one for the comparison to mean anything.
        assert!(
            expected_smoothness > 0.0,
            "fixture regressed to a zero-cost trajectory; seed/best/last become indistinguishable"
        );

        let progress = objective_after_optimize(1.0, 5);
        assert_relative_eq!(progress.seed.smoothness, expected_smoothness, epsilon = EPS);
        assert_relative_eq!(progress.seed.collision, expected_collision, epsilon = EPS);
        assert_relative_eq!(
            progress.seed.total(),
            expected_smoothness + expected_collision,
            epsilon = EPS
        );
    }

    /// The invariant `best` is *built* to satisfy, stated where a drift from
    /// `chomp_optimizer.cpp:338` would trip it: the returned trajectory is
    /// never worse than the seed, and never worse than the final iterate.
    #[test]
    fn objective_best_is_never_above_seed_or_last() {
        for (goal, iterations) in [(1.0, 10), (1.8, 3), (1.8, 50)] {
            let progress = objective_after_optimize(goal, iterations);
            assert!(
                progress.best.total() <= progress.seed.total(),
                "goal {goal}: best {} exceeded seed {}",
                progress.best.total(),
                progress.seed.total()
            );
            assert!(
                progress.best.total() <= progress.last.total(),
                "goal {goal}: best {} exceeded last {}",
                progress.best.total(),
                progress.last.total()
            );
            assert!(progress.improvement() >= 0.0, "goal {goal}");
        }
    }

    /// `last` is a distinct observation from `best`, and its sign is open:
    /// the same optimizer descends on one fixture and climbs above its own
    /// starting point on another. If `last` were a copy of `best` -- the way
    /// upstream's single retained `best_group_trajectory_cost_` is -- the
    /// second case here could not be written at all.
    #[test]
    fn objective_last_can_sit_above_seed_where_best_cannot() {
        let descending = objective_after_optimize(1.0, 50);
        assert!(
            descending.descent() > 0.0,
            "goal 1.0 should descend, got {}",
            descending.descent()
        );
        assert_relative_eq!(
            descending.last.total(),
            descending.best.total(),
            epsilon = EPS
        );

        let climbing = objective_after_optimize(1.8, 3);
        assert!(
            climbing.descent() < 0.0,
            "goal 1.8 should climb above its seed, got descent {}",
            climbing.descent()
        );
        assert!(
            climbing.last.total() > climbing.best.total(),
            "goal 1.8: last {} did not exceed best {}",
            climbing.last.total(),
            climbing.best.total()
        );
        // The case the round exists to make visible: the optimizer ended above
        // where it started, and only the best-snapshot kept the answer from
        // being worse than the input.
        assert_relative_eq!(climbing.best.total(), climbing.seed.total(), epsilon = EPS);
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
            optimizer.iteration, 0,
            "num_collision_free_iterations == 0 must break out on the very first should_break_out \
             pass, and self.iteration's only writer is the loop's own unconditional per-pass \
             increment, which a break skips -- exactly like a for-loop's control variable at break"
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
            0,
            "a threshold strictly above the measured c_cost must break out on the first pass, and \
             a zero grace period means that break is immediate -- self.iteration's only writer, \
             the loop's own unconditional per-pass increment, never runs"
        );
    }

    /// Pins `optimize`'s doc comment's "`self.iteration` advances by exactly
    /// one pass, never two" deviation note. Unlike
    /// `optimize_collision_threshold_break_is_a_strict_less_than` (which
    /// forces `max_iterations_after_collision_free: 0` to get an immediate
    /// break), this uses `ChompParameters::default`'s own grace period
    /// (`5`), so `num_collision_free_iterations` is non-zero when the
    /// collision-threshold branch trips -- `collision_free_iteration`
    /// reaches only `1`, `1 > 5` is false, and the loop does not break.
    /// Upstream advances `iteration_` by 2 on exactly this pass (its own
    /// branch-local `iteration_++` plus the `for` loop's unconditional
    /// step); this port's `self.iteration` has exactly one writer left --
    /// the loop's own unconditional step -- so it advances by 1.
    #[test]
    fn optimize_iteration_advances_by_one_pass_when_the_threshold_branch_fires_without_breaking() {
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
            "this pins the threshold branch firing without breaking, which needs the branch \
             reachable (filter_mode off) and its grace period non-zero -- a zero grace period \
             breaks immediately instead of falling through to the loop's own increment"
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
            optimizer.iteration, 1,
            "an empty env field's c_cost (0.0) is below the default collision_threshold \
             (0.07) from the first pass, so the threshold branch's should_break_out fires \
             on iteration 0 without breaking (grace period 5 > collision_free_iteration 1) -- \
             the loop's own unconditional increment is self.iteration's only writer, putting it \
             at 1 after exactly one real optimization pass, not 2"
        );
    }

    /// Test double for [`DistanceField`] whose
    /// [`DistanceField::distance_gradient`] reports a caller-chosen
    /// distance per `optimize` pass instead of a real geometric lookup --
    /// every other [`DistanceField`] method delegates to a real (empty)
    /// field, so nothing but the collision-cost-driving query is faked.
    /// [`get_collision_sphere_gradients`](cspace_collision::distance_field) is the only
    /// caller in the environment-proximity path, and it queries nothing but
    /// `distance_gradient` (`p.x, p.y, p.z` in, `DistanceGradient` out), so
    /// overriding that one method is sufficient to control `c_cost`.
    ///
    /// Exists to drive a below-threshold-then-above-threshold `c_cost`
    /// sequence through the real `optimize` loop: every real collision
    /// fixture in this file produces a *monotone* sequence (constant ~0, or
    /// one-shot convergence to ~0), so no scenario built from real geometry
    /// can exercise a pass whose collision-threshold condition fires and
    /// then does *not* refire on the very next pass -- exactly the case
    /// that distinguishes "`should_break_out` persists across the whole
    /// call" (upstream, `chomp_optimizer.cpp:300`) from "`should_break_out`
    /// resets every pass" (this port, pre-fix).
    struct StepDistanceField {
        inner: PropagationDistanceField,
        calls: std::cell::Cell<u32>,
        /// How many `distance_gradient` calls pass 0 issues --
        /// `perform_forward_kinematics`'s own `self.iteration == 0` branch
        /// walks every trajectory point, unlike every later pass (free
        /// segment only) -- measured per fixture, not assumed, since it
        /// depends on the fixture's own collision-sphere count.
        calls_pass_zero: u32,
        /// How many `distance_gradient` calls every pass after 0 issues.
        calls_pass_n: u32,
    }

    impl DistanceField for StepDistanceField {
        fn size_x(&self) -> f64 {
            self.inner.size_x()
        }
        fn size_y(&self) -> f64 {
            self.inner.size_y()
        }
        fn size_z(&self) -> f64 {
            self.inner.size_z()
        }
        fn origin_x(&self) -> f64 {
            self.inner.origin_x()
        }
        fn origin_y(&self) -> f64 {
            self.inner.origin_y()
        }
        fn origin_z(&self) -> f64 {
            self.inner.origin_z()
        }
        fn resolution(&self) -> f64 {
            self.inner.resolution()
        }
        fn uninitialized_distance(&self) -> f64 {
            self.inner.uninitialized_distance()
        }
        fn add_points_to_field(&mut self, points: &[Vector3]) {
            self.inner.add_points_to_field(points);
        }
        fn remove_points_from_field(&mut self, points: &[Vector3]) {
            self.inner.remove_points_from_field(points);
        }
        fn update_points_in_field(&mut self, old_points: &[Vector3], new_points: &[Vector3]) {
            self.inner.update_points_in_field(old_points, new_points);
        }
        fn reset(&mut self) {
            self.inner.reset();
        }
        fn distance(&self, x: f64, y: f64, z: f64) -> f64 {
            self.inner.distance(x, y, z)
        }
        fn distance_cell(&self, x: i32, y: i32, z: i32) -> f64 {
            self.inner.distance_cell(x, y, z)
        }
        fn is_cell_valid(&self, x: i32, y: i32, z: i32) -> bool {
            self.inner.is_cell_valid(x, y, z)
        }
        fn num_cells_x(&self) -> i32 {
            self.inner.num_cells_x()
        }
        fn num_cells_y(&self) -> i32 {
            self.inner.num_cells_y()
        }
        fn num_cells_z(&self) -> i32 {
            self.inner.num_cells_z()
        }
        fn grid_to_world(&self, x: i32, y: i32, z: i32) -> Vector3 {
            self.inner.grid_to_world(x, y, z)
        }
        fn world_to_grid(&self, world: &Vector3) -> (bool, i32, i32, i32) {
            self.inner.world_to_grid(world)
        }

        fn distance_gradient(&self, _x: f64, _y: f64, _z: f64) -> DistanceGradient {
            let call = self.calls.get();
            self.calls.set(call + 1);
            let pass = if call < self.calls_pass_zero {
                0
            } else {
                1 + (call - self.calls_pass_zero) / self.calls_pass_n
            };
            // Pass 0 reads as far outside `max_propagation_distance`
            // (0.3, `chomp_collision_field_config`) -- the same "no
            // update" outcome an empty real field gives (its
            // `uninitialized_distance` is that same 0.3, also not `<`
            // it), so pass 0's `c_cost` matches this file's own
            // established empty-field baseline. Every later pass reads as
            // deep in collision.
            let distance = if pass == 0 { 10.0 } else { -1.0 };
            DistanceGradient {
                distance,
                gradient: Vector3::zeros(),
                in_bounds: true,
            }
        }
    }

    /// The `should_break_out` scope bug found against upstream
    /// `chomp_optimizer.cpp:300` (declared once before the `for` loop, never
    /// reset) versus the pre-fix port (declared inside the `while` body,
    /// reset every pass): once any pass sets `should_break_out`,
    /// `collision_free_iteration_` must keep incrementing on *every* later
    /// pass, whether or not that pass's own mesh/threshold condition
    /// refires.
    ///
    /// Built via [`StepDistanceField`] because no real collision fixture in
    /// this file can tell the two apart -- see that type's doc.
    #[test]
    fn optimize_should_break_out_persists_across_iterations_like_upstream() {
        let model = chomp_collision_model();
        // A moving path, not `chomp_full_trajectory`'s degenerate
        // zero-motion one: `get_collision_cost` weights every point's
        // potential by `collision_point_vel_mag`, so a trajectory that
        // never moves reads as ~0 cost regardless of `StepDistanceField`'s
        // reported distance.
        let source = seeded_trajectory(&model, 10, 1.0);
        let start_state = RobotState::new(&model);

        // Measure how many `distance_gradient` queries pass 0 (full
        // trajectory) and every later pass (free segment only) each issue
        // for this fixture, rather than assume it.
        let (calls_pass_zero, calls_pass_n) = {
            let mut cache = chomp_collision_cache(&model);
            let field = StepDistanceField {
                inner: env_field_with_points(&[]),
                calls: std::cell::Cell::new(0),
                calls_pass_zero: u32::MAX,
                calls_pass_n: u32::MAX,
            };
            let parameters = ChompParameters::default();
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
            let calls_pass_zero = field.calls.get();
            optimizer.iteration = 1;
            optimizer
                .perform_forward_kinematics(&mut collision)
                .unwrap();
            let calls_pass_n = field.calls.get() - calls_pass_zero;
            (calls_pass_zero, calls_pass_n)
        };
        assert!(
            calls_pass_zero > 0,
            "fixture must query the environment field on pass 0"
        );
        assert!(
            calls_pass_n > 0,
            "fixture must query the environment field on later passes"
        );

        let mut full = source.clone();
        let mut cache = chomp_collision_cache(&model);
        let field = StepDistanceField {
            inner: env_field_with_points(&[]),
            calls: std::cell::Cell::new(0),
            calls_pass_zero,
            calls_pass_n,
        };
        let mut collision = ChompCollisionContext {
            cache: &mut cache,
            env_distance_field: &field,
        };
        let parameters = ChompParameters {
            max_iterations: 5,
            max_iterations_after_collision_free: 1,
            collision_threshold: 0.5,
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

        let trace = optimizer.loop_trace().unwrap();
        assert_eq!(
            trace.exit,
            ChompExit::BreakOut,
            "collision_free_iteration must keep counting on pass 1 even though pass 1's own \
             c_cost is back above collision_threshold -- upstream's should_break_out \
             (chomp_optimizer.cpp:300) is set on pass 0 and never reset, so pass 1 still \
             enters the should_break_out block and its grace period (1) is exceeded"
        );
        assert_eq!(
            trace.evaluations, 2,
            "must break after pass 1's grace-period check, not run to max_iterations"
        );
    }

    /// `num_collision_free_iterations` has two independent, non-`else`
    /// write sites (`chomp_optimizer.cpp:373`/`410`, mesh-to-mesh and
    /// collision-threshold respectively) that write *different* values
    /// (`0` vs `parameters.max_iterations_after_collision_free`) on a pass
    /// where both fire -- see
    /// `optimize_collision_threshold_no_longer_discards_mesh_to_meshs_immediate_break_signal`
    /// below for that same-pass case and the fix. This control test
    /// isolates the mesh-to-mesh site alone (`filter_mode: true` disables
    /// the collision-threshold site per `chomp_optimizer.cpp:406`'s
    /// `if (!parameters_->filter_mode_)` guard, matching the existing
    /// `optimize_runs_exactly_max_iterations_when_filter_mode_and_mesh_to_mesh_never_break_out`
    /// precedent), confirming its own `num_collision_free_iterations = 0`
    /// signal reaches the break check unmodified and breaks out after
    /// exactly one `should_break_out` pass. See
    /// `optimize_collision_threshold_no_longer_discards_mesh_to_meshs_immediate_break_signal`
    /// for the same mesh-to-mesh signal with the threshold site also
    /// firing, where the outcome now matches this one instead of differing
    /// from it.
    #[test]
    fn optimize_mesh_to_mesh_alone_breaks_out_after_exactly_one_should_break_out_pass() {
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
            .optimize(&mut full, &mut collision, &mut |_, _| true, &mut rng)
            .unwrap();

        assert_eq!(
            optimizer.iteration, 0,
            "mesh-to-mesh alone sets num_collision_free_iterations = 0 on iteration 0, and \
             filter_mode disables the only other writer, so the very next should_break_out \
             check (num_collision_free_iterations == 0) must break immediately -- \
             self.iteration's only writer is the loop's own increment, which a break skips"
        );
    }

    /// The shape 249 of the Phase 8 benchmark's 380 solved problems have
    /// (`PORTING-PLAN.md` §296): the mesh-to-mesh closure is true on the
    /// seed, so the loop leaves after costing exactly one trajectory. The
    /// assertion that carries the finding is `evaluations == 1` paired with
    /// `first_pass_max_update > 0.0` -- an increment *was* computed and
    /// applied, and then nothing ever costed it.
    #[test]
    fn loop_trace_says_one_evaluation_when_the_seed_already_passes_mesh_to_mesh() {
        let model = chomp_collision_model();
        let mut source = chomp_full_trajectory(&model, 10);
        // A kink in the seed, so the smoothness gradient is not identically
        // zero. Without it the applied update is `0.0` for a reason that
        // belongs to the fixture (a constant trajectory is already
        // smoothness-optimal) rather than to the loop being measured.
        let kink = vec![0.3; source.num_joints()];
        source.set_trajectory_point(4, &kink);
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
            .optimize(&mut full, &mut collision, &mut |_, _| true, &mut rng)
            .unwrap();

        let trace = optimizer.loop_trace().expect("optimize records a trace");
        assert_eq!(trace.evaluations, 1);
        assert_eq!(trace.exit, ChompExit::BreakOut);
        assert_eq!(
            trace.mesh_checks, 1,
            "iteration 0 is a multiple of 10, so the mesh check ran exactly once before breaking out"
        );
        assert_eq!(trace.mesh_free_passes, 1);
        assert_eq!(
            trace.threshold_checks, 0,
            "filter_mode disables the comparison itself, not just its firing -- distinct from \
             below_threshold_passes == 0, which alone cannot tell 'never compared' apart from \
             'compared and never fired'"
        );
        assert_eq!(trace.below_threshold_passes, 0, "filter_mode disables it");
        assert_eq!(
            trace.accepted, 0,
            "`accepted` counts the strict improvements after the seed pass, and a loop with one \
             evaluation cannot have one"
        );
        assert!(
            trace.first_pass_max_update > 0.0,
            "the increments for the pass that broke out were still computed and applied; what \
             never happened is a second `perform_forward_kinematics` to cost them"
        );
    }

    /// `chomp_optimizer.cpp:486-490`'s dead detector, alive: the
    /// collision-threshold branch trusts `c_cost` (the sphere/distance-field
    /// term), not a real mesh check, and `best_group_trajectory` can still
    /// be replaced *during* the grace period by a later accept, since
    /// accepts are driven by total cost (smoothness + collision), not mesh
    /// safety. Pass 0 trips the threshold branch on an empty env field
    /// (`c_cost == 0.0`), seeding `best_group_trajectory` unconditionally
    /// (upstream's `iteration_ == 0` arm always does, cost or no cost); the
    /// seed's kink gives pass 1 a nonzero smoothness gradient, so its lower
    /// total cost is an accept that replaces `best_group_trajectory` before
    /// the grace period (`max_iterations_after_collision_free: 1`) expires
    /// on that same pass. The injected mesh closure reports every
    /// trajectory unsafe -- `optimize`'s grace-period-expiry re-check must
    /// correct `is_collision_free` to `false`, not return the threshold
    /// branch's optimistic `true` from two passes back unexamined.
    #[test]
    fn optimize_corrects_is_collision_free_when_the_grace_period_expires_on_a_mesh_unsafe_trajectory()
     {
        let model = chomp_collision_model();
        let mut source = chomp_full_trajectory(&model, 10);
        // Same device as `loop_trace_says_one_evaluation_when_the_seed_already_passes_mesh_to_mesh`:
        // a kink so the smoothness gradient is not identically zero, or
        // pass 1 has nothing to descend and cannot be an accept.
        let kink = vec![0.3; source.num_joints()];
        source.set_trajectory_point(4, &kink);
        let start_state = RobotState::new(&model);
        let mut full = source.clone();
        let mut cache = chomp_collision_cache(&model);
        let field = env_field_with_points(&[]);
        let mut collision = ChompCollisionContext {
            cache: &mut cache,
            env_distance_field: &field,
        };
        let parameters = ChompParameters {
            max_iterations: 2,
            max_iterations_after_collision_free: 1,
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

        let trace = optimizer.loop_trace().expect("optimize records a trace");
        assert_eq!(
            trace.below_threshold_passes, 2,
            "an empty env field keeps c_cost at 0.0, below the default collision_threshold, on \
             both passes"
        );
        assert_eq!(
            trace.accepted, 1,
            "pass 1's lower smoothness cost (from the seed's kink) must be an accept -- that is \
             what replaces best_group_trajectory during the still-open grace period"
        );
        assert_eq!(trace.exit, ChompExit::BreakOut);
        assert!(
            !result,
            "the injected mesh closure reports every trajectory unsafe; the grace-period-expiry \
             re-check must correct is_collision_free to false instead of returning the threshold \
             branch's stale true"
        );
        assert!(!optimizer.is_collision_free());
    }

    /// The other end of `evaluations`: with nothing ever breaking out, the
    /// loop costs one trajectory per iteration up to `max_iterations`, and
    /// the exit reason is the bound rather than the break.
    #[test]
    fn loop_trace_counts_one_evaluation_per_iteration_when_nothing_breaks_out() {
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
            max_iterations: 7,
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

        let trace = optimizer.loop_trace().expect("optimize records a trace");
        assert_eq!(trace.evaluations, 7);
        assert_eq!(trace.exit, ChompExit::IterationBound);
        assert_eq!(
            trace.mesh_checks, 1,
            "iteration 0 is the only multiple of 10 in a 7-iteration run, so the check ran \
             exactly once -- mesh_free_passes == 0 below is 'checked and never found it free', \
             not 'never checked'"
        );
        assert_eq!(trace.mesh_free_passes, 0);
    }

    /// `seed_points_*` name the *first* pass, which is a different claim
    /// from "the counts the last pass happened to leave behind". Two runs
    /// over the same seed and the same obstacle, one costing a single
    /// trajectory and one costing twelve, must report the same seed counts
    /// -- and the twelve-pass run's live counter must have moved off them,
    /// or the two readings would be indistinguishable.
    #[test]
    fn loop_trace_seed_point_counts_are_the_first_passs_not_the_last() {
        let model = chomp_collision_model();
        let source = chomp_full_trajectory(&model, 10);
        let start_state = RobotState::new(&model);

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
                &ChompParameters::default(),
                &start_state,
                &mut collision,
                None,
            )
            .unwrap();
            optimizer
                .perform_forward_kinematics(&mut collision)
                .unwrap();
            optimizer.collision_point_pos_eigen[optimizer.free_vars_start][0]
        };

        let run = |max_iterations: i32| {
            let mut full = source.clone();
            let mut cache = chomp_collision_cache(&model);
            let field = env_field_with_points(&[obstacle_point]);
            let mut collision = ChompCollisionContext {
                cache: &mut cache,
                env_distance_field: &field,
            };
            let parameters = ChompParameters {
                max_iterations,
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
            (
                optimizer.loop_trace().expect("optimize records a trace"),
                optimizer.points_in_collision,
            )
        };

        let (one, _) = run(1);
        let (twelve, last_points_in_collision) = run(12);

        assert_eq!(one.evaluations, 1);
        assert_eq!(twelve.evaluations, 12);
        assert!(
            one.seed_points_in_collision > 0,
            "the obstacle sits on a collision sphere's center, so the seed is in collision"
        );
        assert_eq!(
            one.seed_points_in_collision,
            twelve.seed_points_in_collision
        );
        assert_eq!(
            one.seed_points_within_clearance,
            twelve.seed_points_within_clearance
        );
        assert!(
            one.seed_points_within_clearance > 0,
            "the same obstacle puts points inside their clearance band, which is a different \
             count over a different range -- see the two fields' docs, neither bounds the other"
        );
        assert_ne!(
            twelve.seed_points_in_collision, last_points_in_collision,
            "the twelfth pass left a different count behind, so the seed reading is not just \
             whatever the loop last wrote"
        );
    }

    /// `collision_costs` must be the loop's actual per-pass `c_cost`
    /// sequence, not a placeholder: one entry per evaluated pass, its first
    /// entry matching the independently-read seed objective's collision
    /// term and its last entry matching the independently-read last
    /// objective's collision term. Reusing
    /// `loop_trace_seed_point_counts_are_the_first_passs_not_the_last`'s
    /// obstacle-on-a-collision-sphere fixture so the seed cost is
    /// genuinely nonzero, not a vacuous all-zero pass.
    #[test]
    fn loop_trace_collision_costs_has_one_entry_per_evaluated_pass() {
        let model = chomp_collision_model();
        let source = chomp_full_trajectory(&model, 10);
        let start_state = RobotState::new(&model);

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
                &ChompParameters::default(),
                &start_state,
                &mut collision,
                None,
            )
            .unwrap();
            optimizer
                .perform_forward_kinematics(&mut collision)
                .unwrap();
            optimizer.collision_point_pos_eigen[optimizer.free_vars_start][0]
        };

        let mut full = source.clone();
        let mut cache = chomp_collision_cache(&model);
        let field = env_field_with_points(&[obstacle_point]);
        let mut collision = ChompCollisionContext {
            cache: &mut cache,
            env_distance_field: &field,
        };
        let parameters = ChompParameters {
            max_iterations: 5,
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

        let trace = optimizer.loop_trace().expect("optimize records a trace");
        let progress = optimizer
            .objective()
            .expect("a completed loop always has an objective");

        assert_eq!(
            trace.collision_costs.len(),
            trace.evaluations as usize,
            "one collision_costs entry per evaluated pass"
        );
        assert!(
            trace.collision_costs[0] > 0.0,
            "the obstacle sits on a collision sphere's center, so the seed pass must be in \
             collision"
        );
        assert_eq!(
            trace.collision_costs[0], progress.seed.collision,
            "the first recorded pass must be the same seed collision cost the objective reports"
        );
        assert_eq!(
            *trace.collision_costs.last().unwrap(),
            progress.last.collision,
            "the last recorded pass must be the same last-evaluated collision cost the \
             objective reports"
        );
    }

    /// Item 1 family sweep (an earlier round): the same mesh-to-mesh signal
    /// as
    /// [`optimize_mesh_to_mesh_alone_breaks_out_after_exactly_one_should_break_out_pass`],
    /// but with the collision-threshold site *not* disabled (`filter_mode`
    /// at its `false` default) and an empty env field, whose `c_cost ==
    /// 0.0` is below `collision_threshold` (`0.07`) on every single pass --
    /// so the threshold site also fires every pass, including the first,
    /// where mesh-to-mesh fires too (`self.iteration % 10 == 0` at
    /// `self.iteration == 0`). This is the reachability case: both
    /// conditions are independent (one keys off the pass count, the other
    /// off a computed cost) and this fixture makes both true on pass 0
    /// without needing anything contrived.
    ///
    /// `chomp_optimizer.cpp:373` (mesh-to-mesh) sets
    /// `num_collision_free_iterations_ = 0`; `chomp_optimizer.cpp:410`
    /// (collision-threshold) runs immediately after in the *same* pass.
    /// Upstream writes both unconditionally, so whichever runs second (the
    /// threshold site, textually) wins and overwrites the mesh site's `0`
    /// with `parameters_->max_iterations_after_collision_free_` (default
    /// `5`) -- discarding a ground-truth mesh-safety confirmation
    /// (`isCurrentTrajectoryMeshToMeshCollisionFree`, `chomp_optimizer.cpp:520-537`,
    /// a real `planning_scene_->isPathValid` check on the actual
    /// trajectory) in favor of a sphere/distance-field cost proxy's weaker
    /// one (`getCollisionCost`, `:691-`, which never calls the mesh check
    /// at all). The asymmetry in what each signal actually verifies is why
    /// mesh must win, not just a preference: deferring to the proxy after
    /// the real check already succeeded cannot produce new information,
    /// only more passes for `best_group_trajectory` to be replaced by an
    /// unverified one (the same failure mode
    /// `optimize_corrects_is_collision_free_when_the_grace_period_expires_on_a_mesh_unsafe_trajectory`
    /// closes at the other end, on grace-period expiry).
    ///
    /// Fixed: the threshold site's write to `num_collision_free_iterations`
    /// is now conditional on the mesh site not having already confirmed
    /// safety this same pass, so the mesh signal wins independent of
    /// execution order. `is_collision_free` was already `true` either way
    /// (both sites agree, unaffected by this fix) -- what changes is the
    /// *termination point*: this fixture now breaks after exactly 1
    /// `should_break_out` pass, identical to the mesh-alone control test
    /// above, instead of needing `collision_free_iteration_ > 5`.
    #[test]
    fn optimize_collision_threshold_no_longer_discards_mesh_to_meshs_immediate_break_signal() {
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
            ..ChompParameters::default()
        };
        assert_eq!(
            parameters.max_iterations_after_collision_free, 5,
            "this test's doc contrasts the fixed outcome with what the (larger) default grace \
             period would have produced; if the default ever changes, that contrast must be \
             re-checked"
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

        let result = optimizer
            .optimize(&mut full, &mut collision, &mut |_, _| true, &mut rng)
            .unwrap();

        assert!(
            result,
            "mesh-to-mesh and collision-threshold both report collision-free"
        );
        assert_eq!(
            optimizer.collision_free_iteration, 1,
            "the mesh site's num_collision_free_iterations = 0 must survive the threshold site \
             also firing this same pass, so breaking out needs only collision_free_iteration > 0 \
             -- a regression back to unconditional overwriting would need > 5 instead, like the \
             pre-fix pin this test replaces"
        );
        assert_eq!(
            optimizer.iteration, 0,
            "mesh-to-mesh's num_collision_free_iterations = 0 surviving unmodified must break \
             out after exactly 1 pass, identical to the mesh-alone control test above -- a \
             regression to the old overwrite would run well past it"
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
        // `calculate_smoothness_increments` reaches two `Error::other`
        // sites: its own joint_costs-length guard, and `ChompCost::
        // derivative`'s joint_trajectory-length guard. A bare
        // matches!(err, Error::Other(_)) cannot tell them apart.
        let traj = trajectory(20);
        let costs = joint_costs(&traj, 1e-6);
        let err = calculate_smoothness_increments(&costs[..costs.len() - 1], &traj).unwrap_err();
        assert!(err.to_string().contains("joint_costs has"));
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
        // `calculate_total_increments` has 3 `Error::other` sites (column
        // count vs. joint_costs, row count mismatch between the two
        // increment matrices, and a per-joint quadratic_cost_inverse shape
        // guard); "columns" appears only in this one's message.
        let traj = trajectory(20);
        let costs = joint_costs(&traj, 1e-6);
        let num_free = traj.num_free_points();
        let smoothness = DMatrix::<f64>::zeros(num_free, costs.len());
        let collision = DMatrix::<f64>::zeros(num_free, costs.len() - 1);
        let parameters = ChompParameters::default();
        let err =
            calculate_total_increments(&costs, &smoothness, &collision, &parameters).unwrap_err();
        assert!(err.to_string().contains("columns"));
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
