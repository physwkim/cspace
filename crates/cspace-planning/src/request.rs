// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// No single upstream file: [`PlanningRequest`] replaces the fields of
// `moveit_msgs::msg::MotionPlanRequest` this crate's six request adapters
// actually read (`group_name`, `goal_constraints`, `path_constraints`,
// `workspace_parameters`, `max_velocity_scaling_factor`,
// `max_acceleration_scaling_factor`); [`WorkspaceBounds`] replaces
// `moveit_msgs::msg::WorkspaceParameters` minus its `header` (D1: no ROS
// `std_msgs::msg::Header`).

//! The canonical planning-request type. See the crate doc comment's
//! "Deviation from `cspace_planners::sbp::registry`" section for why this
//! shape, not a transcription of `moveit_msgs::msg::MotionPlanRequest`
//! (which this crate cannot depend on, D1) nor `cspace_planners::sbp`'s
//! existing `PlanningRequest` (a concrete-state goal with RRT-Connect's own
//! tuning fields), is the request type the adapters in this crate operate
//! on.
//!
//! # D8 delta audit (round 21): every field of upstream `MotionPlanRequest`
//!
//! PORTING-PLAN.md §140 confirmed the deeper reason `cspace_planners::sbp`'s
//! own `PlanningRequest`/`PlanningResponse` cannot simply relocate onto
//! these types: the two pairs disagree in shape, not just name. This section
//! is the field-by-field accounting D8 needs before that unification.
//! `third_party/moveit_msgs/msg/MotionPlanRequest.msg` at the pinned commit
//! has 16 fields, one line each below:
//!
//! - `workspace_parameters` (`WorkspaceParameters`) — ported as
//!   [`PlanningRequest::workspace_bounds`] ([`WorkspaceBounds`], minus
//!   `header`, D1).
//! - `start_state` (`RobotState`) — ported as
//!   [`PlanningRequest::start_state`] ([`StartState`]). Round 21 called this
//!   one "distinct: expressed by mutating
//!   [`crate::scene::PlanningScene::current_state_mut`] before the adapter
//!   chain runs". That relocation is real and still how the value reaches
//!   every reader — [`crate::pipeline::generate_plan`] is the single owner
//!   that performs it — but it is not a *substitute* for the field, because
//!   nothing carried the caller's requested overlay as far as that mutation:
//!   a `MotionPlanRequest` arriving over the wire had nowhere to put it, and
//!   `ros/cspace-ros` rejected every non-default `start_state` outright
//!   rather than drop it. See [`crate::start_state`]'s module doc for the
//!   upstream measurement (the field is an overlay on the scene's current
//!   state, not a complete state) and PORTING-PLAN.md §250.4/§254 for the
//!   round trip that rejection blocked.
//! - `goal_constraints` (`Constraints[]`) — ported as
//!   [`PlanningRequest::goal_constraints`] (`Vec<KinematicConstraintSet>`,
//!   already-typed constraint sets in place of raw messages).
//! - `path_constraints` (`Constraints`) — ported as
//!   [`PlanningRequest::path_constraints`] (`Option<KinematicConstraintSet>`,
//!   an empty message becomes `None`).
//! - `trajectory_constraints` (`TrajectoryConstraints`) — ported as
//!   [`PlanningRequest::trajectory_constraints`] (`Vec<KinematicConstraintSet>`,
//!   from `TrajectoryConstraints::constraints: Vec<Constraints>`).
//! - `reference_trajectories` (`GenericTrajectory[]`) — unported, in scope:
//!   confirmed via `rg -n 'req\.|request\.' planning_pipeline.cpp` that
//!   `generatePlan` itself never reads this field (only `trajectory_constraints`
//!   and `planner_id` are), and no adapter or planner in this workspace
//!   reads it either — relevant only to a reference-trajectory-seeded
//!   optimizer (e.g. STOMP/CHOMP) that does not exist in this crate yet,
//!   the same "not ported speculatively" reasoning the crate doc comment
//!   already applies to planner-specific tuning fields.
//! - `pipeline_id` (`string`) — unported, in scope: same reasoning —
//!   `planning_pipeline.cpp` never reads it (it selects *among* pipelines, a
//!   caller/orchestration concern), and this workspace has exactly one
//!   pipeline.
//! - `planner_id` (`string`) — ported as [`PlanningRequest::planner_id`].
//! - `group_name` (`string`) — ported as [`PlanningRequest::group_name`].
//! - `num_planning_attempts` (`int32`) — unported, in scope: not read by
//!   `planning_pipeline.cpp` itself (confirmed by the same `rg` above);
//!   consumed downstream by a `PlannerManager`'s own retry loop, which is
//!   `cspace_planning::planner_registry`'s concern once D8 lands, not this crate's.
//! - `allowed_planning_time` (`float64`) — unported, in scope: same —
//!   consumed by `PlanningContext::solve`'s own timeout, not by
//!   `planning_pipeline.cpp` or any adapter here.
//! - `max_velocity_scaling_factor` (`float64`) — ported as
//!   [`PlanningRequest::max_velocity_scaling_factor`].
//! - `max_acceleration_scaling_factor` (`float64`) — ported as
//!   [`PlanningRequest::max_acceleration_scaling_factor`].
//! - `cartesian_speed_limited_link` (`string`) — distinct: confirmed via
//!   `rg -n 'LimitMaxCartesianLinkSpeed'` over the full `/home/stevek/work/moveit2`
//!   checkout that no such request-adapter plugin exists anywhere upstream
//!   at the pinned commit, despite the `.msg` file's own comment naming one
//!   — the only real consumers of this field are `pilz_industrial_motion_planner`'s
//!   own trajectory generators (`trajectory_generator_{circ,lin,polyline}.cpp`),
//!   a specific planner plugin outside this crate's default-adapter-chain
//!   scope entirely, not a `default_planning_request_adapters` plugin.
//! - `max_cartesian_speed` (`float64`) — distinct: same reasoning as
//!   `cartesian_speed_limited_link`, same pilz-only consumer.
//! - `smoothness_level` (`float64`) — distinct: same reasoning, same
//!   pilz-only consumer.
//!
//! Total: 9 ported, 3 distinct, 4 unported-in-scope = 16, matching the
//! `.msg` field count exactly.
//!
//! The *normalization* upstream applies to two of those fields —
//! `num_planning_attempts` and `allowed_planning_time` — before any solve is
//! a separate question from the fields themselves, and it is **not ported**.
//! `PlanningContext::setMotionPlanRequest`
//! (`moveit_core/planning_interface/src/planning_interface.cpp:92-103`:
//! `allowed_planning_time <= 0.0` becomes `1.0`, `num_planning_attempts`
//! becomes `std::max(1, n)`) has no counterpart here, by the decision in
//! `PORTING-PLAN.md` §236. The short reason: this port's planning budget is
//! `cspace_planners::sbp::Termination` (`Iterations(usize)` /
//! `Deadline(Duration)` / `Both`, with no `Default` and no "unset" variant),
//! so negative, unset and NaN are all unconstructible rather than repaired,
//! and the upstream guard is NaN-blind in any case — `doc/upstream-bugs.md`'s
//! `set-motion-plan-request-time-guard-polarity`. That decides the rule, not
//! the two fields, which stay unported-in-scope above; §236's expiry
//! tripwires are the two
//! `*_boundaries_are_not_observable_on_the_core_request` tests in
//! `ros/cspace-ros/src/planning.rs`.
//!
//! # D8 delta audit: `cspace_planners::sbp::registry::PlanningRequest`/`PlanningResponse`
//!
//! Read-only (that file is `cspace_planners::sbp`'s, not this crate's — see
//! `crates/cspace-planners/src/sbp/registry.rs:136-166`). Mapping each of
//! its fields onto the canonical types above, in the three buckets D8 needs:
//!
//! **Missing from canonical, D8 must add:** none found this round beyond
//! what round 20 already added (`trajectory_constraints`, `planner_id`) —
//! every sbp-local field below either already has a canonical counterpart or
//! is planner-tuning that canonical deliberately excludes (next bucket).
//!
//! **sbp-local-only, stays off `PlanningRequest` by design (not a gap):**
//! `resolution` (`f64`, `DiscreteMotionValidator` bisection step),
//! `seed` (`u64`, RNG seed), `params` (`RrtConnectParams`) — all three are
//! RRT-Connect-specific tuning; the crate doc comment already documents why
//! `PlanningRequest` deliberately excludes planner tuning (it belongs on
//! each concrete `PlannerManager`-analogous type's own construction, not on
//! a request shape meant to serve more than one planner algorithm). D8
//! moves these onto `RrtConnectManager`'s own construction, not onto
//! `PlanningRequest`.
//!
//! **Different representation, needs conversion — the actual D8 work:**
//! - `goal: Vec<CompoundValue>` (one concrete joint-space state) versus
//!   canonical [`PlanningRequest::goal_constraints`] (`Vec<KinematicConstraintSet>`,
//!   a region). This is the gap PORTING-PLAN.md §140.2 names as the root
//!   cause the two `PlanningRequest` types diverged in the first place.
//!   `registry.rs`'s own module doc already records that closing it needs
//!   two things, of which only the first exists today: (1) a constraint
//!   sampler turning a region into candidate concrete states — ported, as
//!   [`crate::constraints::JointConstraintSampler`]/[`crate::constraints::IkConstraintSamplerAdapter`]/
//!   [`crate::constraints::UnionConstraintSampler`] — and (2)
//!   `crate::rrt_connect::rrt_connect` itself accepting something
//!   `GoalSampleableRegion`-shaped instead of one fixed `S::State` — not
//!   done. D8 depends on p1-robotmodel's in-flight `ConstraintSamplerManager`
//!   wiring (PORTING-PLAN.md §140.2's stated precondition) landing first.
//! - sbp-local `PlanningResponse::trajectory: Vec<RobotState<'m>>` (bare
//!   waypoints) versus canonical [`crate::PlanningResponse::trajectory`]
//!   (`RobotTrajectory<'m>`, one `duration_from_previous` per waypoint) —
//!   mechanical, not a design question: `RobotTrajectory::new` +
//!   `add_suffix_way_point(state, 0.0)` per waypoint reproduces the
//!   bare-waypoint shape with an explicit zero/unset duration, exactly what
//!   this crate's own response adapters
//!   ([`crate::response_adapters::AddRuckigTrajectorySmoothing`]/
//!   [`crate::response_adapters::AddTimeOptimalParameterization`]) already
//!   expect to receive and fill in.
//! - sbp-local response has no `planner_id`; canonical requires one — D8
//!   either has `RrtConnectContext::solve` fill in `"rrt_connect"` directly,
//!   or leaves it empty and relies on `crate::pipeline::generate_plan`'s
//!   existing fallback from [`PlanningRequest::planner_id`]. Mechanical
//!   addition, not a conversion of existing data.
//! - `path_constraints: Option<KinematicConstraintSet>` and `group_name:
//!   String` are already byte-for-byte the same type on both sides — no
//!   conversion needed, direct reuse.

use crate::constraints::KinematicConstraintSet;
use cspace_core::geometry::Vector3;

use crate::start_state::StartState;

/// Replaces `moveit_msgs::msg::WorkspaceParameters` (minus `header`, D1): the
/// axis-aligned box a sampling-based planner should search within.
///
/// `Default` is the all-zero box, matching an unset ROS message field. What
/// counts as unset is [`WorkspaceBounds::is_unspecified`], which is *wider*
/// than equality with this default — see that method.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct WorkspaceBounds {
    /// `min_corner`.
    pub min_corner: Vector3,
    /// `max_corner`.
    pub max_corner: Vector3,
}

impl Default for WorkspaceBounds {
    fn default() -> Self {
        Self {
            min_corner: Vector3::zeros(),
            max_corner: Vector3::zeros(),
        }
    }
}

impl WorkspaceBounds {
    /// Upstream's "the planning volume was not specified" test
    /// (`validate_workspace_bounds.cpp:77-82`): all six corner components
    /// below `DBL_EPSILON` *in magnitude*, six independent `std::abs(v) <
    /// epsilon` comparisons.
    ///
    /// Not `*self == Self::default()`. The two agree on the all-zero box
    /// every real caller sends, and disagree on a box whose corners are
    /// nonzero but under `2.22e-16`: upstream calls that unspecified and
    /// substitutes the default cube, where exact equality would keep it and
    /// hand a planner a degenerate sampling volume. Named here rather than
    /// spelled at the one call site so the rule cannot be re-derived as an
    /// equality by the next reader — the two are not interchangeable, and
    /// the all-zero `Default` above is what makes them look it.
    ///
    /// `NaN` is not unspecified, on either side: `std::abs(NaN) < epsilon`
    /// is false, and so is the `<` below.
    pub fn is_unspecified(&self) -> bool {
        self.min_corner
            .iter()
            .chain(self.max_corner.iter())
            .all(|v| v.abs() < f64::EPSILON)
    }
}

/// A motion planning query, in the shape this crate's request adapters (and,
/// through them, [`crate::run_request_adapters`]) operate on.
///
/// See the crate doc comment for why `goal_constraints` is
/// `Vec<KinematicConstraintSet>` rather than a concrete state, and why
/// planner-specific tuning (RRT-Connect's step size, STOMP's iteration
/// count, ...) is deliberately not a field here.
///
/// `Default` fills [`PlanningRequest::trajectory_constraints`] with an empty
/// `Vec`, [`PlanningRequest::planner_id`] with `""` and
/// [`PlanningRequest::start_state`] with [`StartState::CurrentState`],
/// matching an unset `moveit_msgs::msg::MotionPlanRequest` field for all
/// three — the same unset-means-default reading [`WorkspaceBounds::default`]
/// already documents for [`PlanningRequest::workspace_bounds`].
#[derive(Debug, Clone, Default)]
pub struct PlanningRequest {
    /// The [`cspace_core::model::JointModelGroup`] to plan for.
    pub group_name: String,
    /// The state to plan *from*, as an overlay on the scene's current state —
    /// see [`StartState`] for why an overlay and not a complete state, and
    /// [`crate::pipeline::generate_plan`]'s module doc, "Semantic 7", for the
    /// single site that applies it.
    pub start_state: StartState,
    /// Candidate goal constraint sets — a state satisfying *any one* set is
    /// an acceptable goal. Matches
    /// `MotionPlanRequest::goal_constraints: Vec<Constraints>`'s
    /// any-of-these-sets contract exactly (`planning_scene.cpp`'s own
    /// `isPathValid` reads it the same way — see
    /// [`crate::scene::PlanningScene::is_path_valid`]'s `goal_constraints`
    /// parameter).
    pub goal_constraints: Vec<KinematicConstraintSet>,
    /// Constraints every waypoint (not just the goal) must satisfy. `None`
    /// means unconstrained.
    pub path_constraints: Option<KinematicConstraintSet>,
    /// The box a sampling-based planner should search within.
    /// [`crate::request_adapters::ValidateWorkspaceBounds`] fills this in
    /// from a default when left at [`WorkspaceBounds::default`].
    pub workspace_bounds: WorkspaceBounds,
    /// A factor in `(0, 1]` scaling every joint's velocity limit. Read by
    /// [`crate::response_adapters::AddRuckigTrajectorySmoothing`]/
    /// [`crate::response_adapters::AddTimeOptimalParameterization`], exactly
    /// as upstream's identically-named field is.
    pub max_velocity_scaling_factor: f64,
    /// A factor in `(0, 1]` scaling every joint's acceleration limit. Same
    /// readers as [`PlanningRequest::max_velocity_scaling_factor`].
    pub max_acceleration_scaling_factor: f64,
    /// Per-waypoint joint-position constraints a planner chain feeds
    /// forward from one planner's successful trajectory into the next
    /// planner's request — see [`crate::pipeline`]'s module doc, "Semantic
    /// 1: planner-chain feedforward". Empty unless [`crate::pipeline::generate_plan`]
    /// (or a caller replicating it) has already run at least one planner.
    /// Not read by any request adapter in this crate; upstream's identically
    /// named `MotionPlanRequest::trajectory_constraints` is the same shape
    /// for the same reason.
    pub trajectory_constraints: Vec<KinematicConstraintSet>,
    /// Which planner produced (or should produce) [`crate::PlanningResponse`].
    /// Read by [`crate::pipeline::generate_plan`] only as the fallback value
    /// for [`crate::PlanningResponse::planner_id`] when a planner leaves
    /// that field empty — see [`crate::pipeline`]'s module doc, "Semantic 4:
    /// `planner_id` fallback". Not otherwise interpreted by this crate: which
    /// string names which planner is a caller/registry concern.
    pub planner_id: String,
}
