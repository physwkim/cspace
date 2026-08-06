// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// No upstream file ported line-for-line: this is a D1/D4-adapted stand-in
// for the plugin half of
//   moveit_core/planning_interface/include/moveit/planning_interface/planning_interface.hpp
// The interface itself (`PlannerManager`, `PlanningContext`,
// `MotionPlanRequest`, `MotionPlanResponse`) lives in `moveit-planning`,
// mirroring the single upstream package that declares all four; this file
// holds one implementation of it and that implementation's registration.

//! One concrete planner, [`RrtConnectManager`], implementing
//! [`moveit_planning::PlannerManager`] over a real
//! [`moveit_scene::PlanningScene`] via
//! [`crate::planning_scene_validity::PlanningSceneValidityChecker`].
//!
//! # The request type is `moveit-planning`'s, not this crate's (D8/§140)
//!
//! Until this round this file declared its own `PlanningRequest`,
//! `PlanningResponse`, `PlanningContext` and `PlannerManager`. They shared
//! only *names* with `moveit-planning`'s, which meant the workspace's
//! planning pipeline could not call the workspace's only planner at all:
//! `moveit_planning::pipeline::generate_plan` wanted
//! `moveit_planning::PlanningRequest` and this crate accepted something
//! else with the same spelling. All four names now resolve to
//! `moveit-planning`'s single set, and this crate depends on that crate.
//!
//! The three fields the old local request had that upstream's
//! `MotionPlanRequest` does not — `resolution`, `seed` and `params` — moved
//! onto [`RrtConnectManager`] itself, together with `solver`. That is
//! upstream's own placement, not a compromise: per-planner tuning lives in
//! `PlannerConfigurationSettings`/`setPlannerConfigurations`
//! (`planning_interface.hpp:56-72,193`), owned by the `PlannerManager`,
//! while `MotionPlanRequest` carries only what a *caller* asks for. They
//! stay concretely typed here, and upstream's stringly typed
//! `config: map<string, string>` bag arrives *beside* them as
//! [`RrtConnectManager::configurations`] — the two are not alternatives.
//! This paragraph used to end "there is no runtime plugin boundary for a
//! string bag to cross", which named the wrong boundary: D4 removes the
//! *plugin* boundary, and the boundary the bag actually crosses is
//! `/set_planner_params`, a service whose values exist only at runtime.
//! The typed fields are the floor; the bag overlays it per query
//! (PORTING-PLAN.md §285).
//!
//! # Goals are constraints; a concrete state is expressed as constraints
//!
//! Upstream's `MotionPlanRequest::goal_constraints` is
//! `Vec<moveit_msgs::msg::Constraints>` — goal *regions*, turned into
//! candidate joint-space states at plan time by a `constraint_samplers`
//! sampler. `moveit_planning::PlanningRequest::goal_constraints` is that,
//! as `Vec<KinematicConstraintSet>`.
//!
//! The old local request instead had a `Goal` enum whose `State` variant
//! carried one concrete `Vec<CompoundValue>`, because
//! [`crate::rrt_connect::rrt_connect`] takes exactly one `S::State` as its
//! goal and nothing existed to resolve a region down to one. That is no
//! longer true (`Goal::Constraints` and `crate::goal_sampler::sample_goal`
//! landed in round 21), and upstream has no counterpart to the `State`
//! variant at all: a caller who wants to reach one specific configuration
//! calls `constructGoalConstraints(state, jmg, tolerance)`
//! (`kinematic_constraints/utils.hpp:99`), which builds one
//! `JointConstraint` per group variable at that state's positions. This
//! port has that function as
//! [`moveit_constraints::utils::construct_goal_joint_constraints`], and every
//! former `Goal::State` caller now goes through it. The behavioural
//! consequence is real and worth stating plainly: a state goal is now
//! reached to within the tolerance the caller passes, not bit-exactly,
//! because it is resolved by sampling the constraint window like any other
//! goal region.
//!
//! `goal_constraints` being a *list* is upstream's any-of semantics
//! (`ModelBasedPlanningContext::setGoalConstraints`,
//! `model_based_planning_context.cpp:664-695`, builds one
//! `ConstrainedGoalSampler` per non-empty set and wraps more than one in a
//! `GoalSampleableRegionMux` whose `sampleGoal` round-robins over them,
//! `detail/goal_union.cpp:83-95`). [`moveit_planning::PlanningContext::solve`] reproduces
//! the any-of rule at the resolution this port samples at: it tries each
//! non-empty set in order and takes the first that yields an accepted
//! state. Empty sets are dropped exactly as upstream drops them
//! (`:679-683`); an all-empty `goal_constraints` is
//! [`PlanError::NoGoalConstraints`], upstream's
//! `INVALID_GOAL_CONSTRAINTS` return at `:690-694`.
//!
//! # What a caller still cannot express
//!
//! A pose (position/orientation) goal reaches a real `IKConstraintSampler`
//! only if the caller supplies a [`moveit_kinematics::KinematicsSolver`] —
//! see [`RrtConnectManager::solver`]. With no solver, a Cartesian-only goal
//! region builds no sampler and falls back to
//! [`crate::space::StateSpace::sample_uniform`] every attempt: not
//! incorrect (the constraint set still gates acceptance), just practically
//! unable to find a tight region by chance within
//! `DEFAULT_MAX_GOAL_SAMPLING_ATTEMPTS` tries. Nothing in this crate picks
//! a solver by name, matching D4's standing exclusion of that
//! runtime-configuration layer (§68.4/§77.1, reaffirmed by §163's D12
//! rejection).
//!
//! # How the sampler wiring got here
//!
//! **Round 14** recorded that `constraint_samplers` was unported; it since
//! landed in `moveit-constraints`:
//!
//! - `ConstraintSampler` (the base trait) and `JointConstraintSampler` ->
//!   ported as [`moveit_constraints::ConstraintSampler`]/
//!   [`moveit_constraints::JointConstraintSampler`],
//!   `moveit-constraints/src/sampler.rs`.
//! - `UnionConstraintSampler` -> ported as
//!   [`moveit_constraints::UnionConstraintSampler`],
//!   `moveit-constraints/src/sampler.rs`, composing other samplers by
//!   sorted dependency order.
//! - `IKConstraintSampler` and `ConstraintSamplerManager::selectDefaultSampler`
//!   -> ported as [`moveit_constraints::IkConstraintSampler`]/
//!   [`moveit_constraints::IkConstraintSamplerAdapter`]
//!   (`moveit-constraints/src/ik_sampler.rs`) and
//!   [`moveit_constraints::select_default_sampler`]
//!   (`moveit-constraints/src/constraint_sampler_manager.rs`). The
//!   dependency edge this needed, `moveit-constraints -> moveit-kinematics`,
//!   now exists (`moveit-constraints/Cargo.toml`'s
//!   `moveit-kinematics.workspace = true`) — no cycle resulted, matching
//!   the no-cycle check this section originally reasoned through before the
//!   edge was added.
//! - `constraint_sampler_tools.{hpp,cpp}` -> still excluded outright:
//!   `visualizeDistribution`'s two overloads need a
//!   `visualization_msgs::msg::MarkerArray` (D1, matching this workspace's
//!   existing `getMarkers()` exclusion), and `countSamplesPerSecond`'s two
//!   overloads are a benchmarking helper with no test or caller needing it.
//!
//! Porting the sampler alone did not close the *goal* half of the
//! capability gap above by itself: [`crate::rrt_connect::rrt_connect`]'s
//! `goal` parameter is one fixed `S::State`, not a region or a
//! re-sampleable source, so even with `IkConstraintSamplerAdapter`
//! available, RRT-Connect's *goal* needed a second change — something to
//! resolve a region down to the one state `rrt_connect` takes — before it
//! could consume a sampler's candidates at all.
//!
//! **Round 21** made that second change: a goal expressed as a
//! [`KinematicConstraintSet`] (mirroring
//! `ompl_interface::ConstrainedGoalSampler`,
//! `crate::goal_sampler::sample_goal` — see that module's own doc comment
//! for exactly what is and is not ported), resolved to one concrete state
//! before [`crate::rrt_connect::rrt_connect`] starts searching. This closes
//! the joint-constraint case fully: a goal set whose
//! [`moveit_constraints::JointConstraint`]s cover every one of the group's
//! variables gets a real [`moveit_constraints::JointConstraintSampler`]
//! (`select_default_sampler`'s Step A) — which is also why a
//! [`moveit_constraints::utils::construct_goal_joint_constraints`] state goal
//! resolves accurately rather than by luck. Through round 22 it did not
//! close the Cartesian-pose case at all: `RrtConnectContext::solve` always
//! passed `solver: None` to `select_default_sampler`, so a
//! [`moveit_constraints::PositionConstraint`]/
//! [`moveit_constraints::OrientationConstraint`]-only goal set
//! built no sampler and fell back to
//! [`crate::space::StateSpace::sample_uniform`] every attempt — not
//! incorrect (the set's own `decide()` still gated acceptance), just
//! practically unable to find a tight Cartesian region by chance within
//! `DEFAULT_MAX_GOAL_SAMPLING_ATTEMPTS` tries.
//!
//! **Round 23** (`PORTING-PLAN.md` §163.3, closing the D12-rejection
//! follow-up §163 left open) added the solver field: a caller
//! who explicitly constructs a
//! [`moveit_kinematics::KinematicsSolver`] (e.g. from
//! `moveit_kinematics::KINEMATICS_SOLVERS`) and sets it
//! now gets a real `IKConstraintSampler` for a Cartesian-pose
//! goal, the same way a full joint-constraint goal already
//! did. `None` is unchanged: identical fallback to uniform sampling.
//! Through round 23 the `path_constraints` `select_default_sampler` call
//! did **not** read it: it was
//! consumed via `.take()` inside the goal call only, so the field's meaning
//! ("the caller's solver") silently depended on which call site executed —
//! a path-constraint region needing IK-backed sampling got none, with no
//! type-level signal that this gap existed.
//!
//! **Round 24** (closing the same `PORTING-PLAN.md` §163.3 gap Round 23
//! opened) fixed that: both
//! `select_default_sampler` calls — `path_constraints`' own and the goal's
//! own — go through the same `resolve_constraint_sampler` helper (below,
//! right before `RrtConnectContext`'s `impl`), wrapped in
//! `SharedKinematicsSolver`,
//! so both are backed by the *same* solver instance instead of only
//! whichever call site got to `.take()` it first. See
//! [`RrtConnectManager::solver`]'s own doc comment for why `Rc<RefCell<>>`
//! was chosen over `Arc` and over splitting the field in two. Since D8
//! moved that field onto the manager, the sharing is established once at
//! construction rather than re-derived per query, and the `.take()` this
//! round's predecessor had to work around is gone entirely: nothing
//! mutates the request.
//!
//! Boundary-tested at the `resolve_constraint_sampler` level
//! (`path_constraints_solver_wiring_matches_the_call_site`, this module's
//! tests) *and*, since round 25's seeding-gap fix
//! (`constrained_sampler::GroupConstraintSampler`'s own doc comment),
//! end-to-end (`path_constraints_end_to_end_wired_vs_unwired`): a wired
//! path sampler now measurably beats an unwired one on a Cartesian-corridor
//! query tight enough to be discriminating.
//!
//! `moveit_planning::PlanningRequest::path_constraints` is carried directly
//! as a [`KinematicConstraintSet`], because path constraints are evaluated
//! per-candidate via `decide()` — see
//! [`crate::planning_scene_validity::PlanningSceneValidityChecker`] — so
//! correctness never depended on a sampler. **Round 20**: `path_constraints`
//! is now also fed to `select_default_sampler` and wired into
//! [`crate::rrt_connect::rrt_connect`]'s uniform-sampling step (not its
//! fixed `goal` — see [`crate::rrt_connect::Sampler`] and
//! [`crate::rrt_connect::ConstrainedStateSampler`], mirroring upstream's
//! `ompl_interface::ConstrainedSampler`), a distinct seam from the goal-region
//! sampling the paragraph above now also describes:
//! [`moveit_planning::PlanningContext::solve`] builds the sampler through
//! `crate::constrained_sampler::GroupConstraintSampler` whenever
//! `path_constraints` is `Some`, purely as a sampling-efficiency aid —
//! `checker` below still enforces the constraint on every candidate
//! regardless of whether a sampler was available to help find one.
//!
//! `start` is not a request field: [`RrtConnectManager::get_planning_context`]
//! reads it from the [`moveit_scene::PlanningScene`] it is given
//! (`scene.current_state()`), matching how upstream planning normally seeds
//! from the scene's current state rather than duplicating it into the
//! request. `moveit_planning::PlanningResponse::start_state` records what
//! that start actually was.

use std::cell::RefCell;
use std::rc::Rc;

use moveit_collision::{CollisionRequest, ParryCollisionEnv};
use moveit_constraints::{
    Constraint, ConstraintSampler, KinematicConstraintSet, select_default_sampler,
};
use moveit_geometry::Isometry3;
use moveit_kinematics::{KinematicsSolver, SolveOptions};
use moveit_model::RobotModel;
use moveit_planner_registry::{PLANNER_MANAGERS, PlannerRegistration};
use moveit_planning::{
    PlannerConfigurationMap, PlannerManager, PlanningContext, PlanningRequest, PlanningResponse,
};
use moveit_scene::PlanningScene;
use moveit_trajectory::RobotTrajectory;
use rand::SeedableRng;
use rand_chacha::ChaCha8Rng;

use crate::constrained_sampler::GroupConstraintSampler;
use crate::error::SbpError;
use crate::joint_model_group_space::JointModelGroupSpace;
use crate::planning_scene_validity::PlanningSceneValidityChecker;
use crate::rrt_connect::{
    ConstrainedStateSampler, PlanningFailure, RrtConnectParams, Sampler, Termination, rrt_connect,
};
use crate::validity::DiscreteMotionValidator;

/// The `max_attempts` every live upstream call to `ConstraintSampler::sample`
/// actually passes: `ModelBasedPlanningContext::getMaximumStateSamplingAttempts()`,
/// configured to `4` by `PlanningContextManager`'s constructor
/// (`planning_context_manager.cpp:259`, `max_state_sampling_attempts_(4)`)
/// and consumed both by path-constraint sampling
/// (`detail/constrained_sampler.cpp:69-70`,
/// `constraint_sampler_->sample(work_state_, ..., getMaximumStateSamplingAttempts())`)
/// and by goal-constraint sampling (`detail/constrained_goal_sampler.cpp:137`).
/// Round 20 used [`moveit_constraints::DEFAULT_MAX_SAMPLING_ATTEMPTS`] (`2`)
/// here instead — that constant is upstream `ConstraintSampler::DEFAULT_MAX_SAMPLING_ATTEMPTS`
/// (`constraint_sampler.hpp:64`), a default-argument fallback for two
/// `sample()` overloads this port's own trait design already collapses away
/// (see `moveit_constraints::sampler`'s doc comment); no live production
/// call site upstream ever actually receives `2`. This constant is the
/// correct one instead.
const DEFAULT_MAX_STATE_SAMPLING_ATTEMPTS: u32 = 4;

/// The outer retry budget [`crate::goal_sampler::sample_goal`] draws
/// against: upstream `ModelBasedPlanningContext::getMaximumGoalSamplingAttempts()`,
/// configured to `1000` by `PlanningContextManager`'s constructor
/// (`planning_context_manager.cpp:260`, `max_goal_sampling_attempts_(1000)`)
/// and consumed as `sampleUsingConstraintSampler`'s own `max_attempts`
/// (`detail/constrained_goal_sampler.cpp:98,102,114`).
const DEFAULT_MAX_GOAL_SAMPLING_ATTEMPTS: u32 = 1000;

// Disposition of `PlanningContextManager`'s remaining three siblings from
// the same constructor initializer list (`planning_context_manager.cpp:258-262`)
// that DEFAULT_MAX_STATE_SAMPLING_ATTEMPTS and DEFAULT_MAX_GOAL_SAMPLING_ATTEMPTS
// above do not already cover, recorded explicitly per-constant rather than
// silently:
//
// - `max_goal_samples_` (`10`, `:258`, `max_goal_samples_(10)`) — not
//   ported. Consumed at `detail/constrained_goal_sampler.cpp:106`
//   (`gls->getStateCount() >= planning_context_->getMaximumGoalSamples()`)
//   to cap how many *accepted* goal states `ob::GoalLazySamples`'s
//   background sampling thread collects before it stops growing the goal
//   region. `getStateCount()` is `GoalLazySamples`'s own state — a
//   multi-goal accumulation this port has no layer for, since
//   `rrt_connect::rrt_connect` roots one goal tree on a single concrete
//   state (see `goal_sampler::sample_goal`'s "Why one state, not a
//   lazily-grown region" doc for the full argument).
//   `goal_sampler::sample_goal` returns after the first accepted sample;
//   there is nothing here for a cap to bound.
// - `max_planning_threads_` (`4`, `:261`) — out of scope for goal
//   sampling: it sizes OMPL's `ompl::tools::ParallelPlan` thread pool,
//   which this port's single-threaded `rrt_connect::rrt_connect` has no
//   equivalent of.
// - `max_solution_segment_length_` (`0.0`, `:262`) — out of scope for
//   goal sampling: it configures post-solve waypoint interpolation
//   spacing (`ModelBasedPlanningContext::simplifySolution`), unrelated to
//   how a goal state or region is sampled.

/// Everything that can go wrong building or running a
/// [`moveit_planning::PlanningContext`] from this crate. Boxed into
/// [`moveit_planning::PlanError`] at the trait boundary, which is
/// deliberately opaque so `moveit-planning` need not depend on any concrete
/// planner crate.
#[derive(Debug, thiserror::Error)]
#[non_exhaustive]
pub enum PlanError {
    /// [`RrtConnectManager::get_planning_context`] was given a
    /// `group_name` the scene's `RobotModel` does not
    /// have, or another boundary-input error from this crate.
    #[error(transparent)]
    Sbp(#[from] SbpError),
    /// [`moveit_planning::PlanningContext::solve`] ran but did not find a path.
    #[error("planning failed: {0}")]
    Failed(#[from] PlanningFailure),
    /// No goal set in `goal_constraints` could be resolved to a single
    /// concrete state within `DEFAULT_MAX_GOAL_SAMPLING_ATTEMPTS` attempts.
    /// Mirrors `ConstrainedGoalSampler::sampleUsingConstraintSampler`
    /// returning `false` after `attempts_so_far >= max_attempts`
    /// (`constrained_goal_sampler.cpp:102-103`) — upstream surfaces that
    /// through `GoalLazySamples`'s own empty-goal-region timeout deep in
    /// OMPL; this port reports it directly instead. Reported only after
    /// *every* set has been tried, matching `GoalSampleableRegionMux`'s
    /// round-robin over all of them (`detail/goal_union.cpp:83-95`).
    #[error("no goal state satisfying the goal constraints was found within the sampling budget")]
    NoGoalSample,
    /// `goal_constraints` held no non-empty
    /// [`KinematicConstraintSet`]. Ports
    /// `ModelBasedPlanningContext::setGoalConstraints` returning
    /// `INVALID_GOAL_CONSTRAINTS` when every set it was given turned out
    /// empty (`model_based_planning_context.cpp:690-694`); upstream drops
    /// empty sets one by one at `:679-683` and only fails if nothing
    /// survives, which is exactly the boundary this variant marks.
    #[error("the request carried no non-empty goal constraint set")]
    NoGoalConstraints,
    /// Assembling the solved path into a
    /// [`moveit_trajectory::RobotTrajectory`] failed. Structurally
    /// unreachable today — the group name was already resolved by
    /// [`RrtConnectManager::get_planning_context`] and every waypoint is a
    /// clone of the scene's own current state — but
    /// [`moveit_trajectory::RobotTrajectory`]'s API is fallible and this
    /// port surfaces that rather than `.expect()`-ing an invariant it holds
    /// only by construction elsewhere.
    #[error("assembling the planned trajectory failed: {0}")]
    Trajectory(#[from] moveit_error::Error),
}

/// [`moveit_planning::PlannerManager`] for
/// [`crate::rrt_connect::rrt_connect`].
///
/// # Why the tuning lives here and not in the request
///
/// [`moveit_planning::PlanningRequest`] is a port of upstream's
/// `MotionPlanRequest`, which carries what a *caller* asks for and nothing
/// about how a particular planner goes about it. Upstream keeps per-planner
/// tuning in `PlannerConfigurationSettings`, handed to the manager through
/// `setPlannerConfigurations` (`planning_interface.hpp:56-72,193`) — so
/// these four fields sit on the manager for the same reason, not as a
/// leftover from the request type they used to live on.
///
/// # Two ways to tune it, and which upstream each one is
///
/// [`RrtConnectManager::configurations`] is the wire-settable half: the
/// [`moveit_planning::PlannerConfigurationMap`]
/// `moveit_planner_registry::PlannerRegistration::construct` hands over,
/// which for a node is whatever `/set_planner_params` has written. It is
/// consulted per query, and the one key it carries here is
/// [`RANGE_KEY`]. The typed fields are the compiled-in defaults it overlays
/// — upstream's equivalent floor is the OMPL planner's own constructor
/// defaults, since a key absent from `ompl_planning.yaml` is simply never
/// passed to `params().setParams` (`planning_context_manager.cpp:213`).
///
/// A caller with no ROS in the picture still constructs
/// [`RrtConnectManager`] directly and sets the typed fields; that is the
/// only way to reach the three ([`RrtConnectManager::seed`],
/// [`RrtConnectManager::solver`], and the rest of
/// [`RrtConnectManager::params`]) that no configuration key maps onto.
pub struct RrtConnectManager {
    /// [`crate::validity::DiscreteMotionValidator`]'s bisection resolution,
    /// in the group's own [`crate::space::StateSpace::distance`] units.
    /// Upstream's nearest equivalent is OMPL's
    /// `longest_valid_segment_fraction` (`model_based_planning_context.cpp:294-315`),
    /// which is a *fraction* of the space's maximum extent rather than an
    /// absolute distance; this port validates against an absolute one
    /// because [`crate::validity::DiscreteMotionValidator`] has no state
    /// space to ask for an extent.
    pub resolution: f64,
    /// Seeds each query's RNG — see [`crate::rrt_connect::rrt_connect`]'s
    /// determinism guarantee under [`crate::rrt_connect::Termination::Iterations`].
    /// Every query this manager builds a context for uses the same seed, so
    /// two identical queries against an identical scene return identical
    /// paths; a caller wanting different draws per query constructs a
    /// manager per query.
    pub seed: u64,
    /// RRT-Connect's own tuning parameters.
    pub params: RrtConnectParams,
    /// Backs *every* [`select_default_sampler`] call
    /// [`moveit_planning::PlanningContext::solve`] makes on the context
    /// this manager builds — the goal's own and
    /// `path_constraints`' own — with one real IK solver,
    /// so a [`moveit_constraints::PositionConstraint`]/
    /// [`moveit_constraints::OrientationConstraint`] region gets a real
    /// `IKConstraintSampler` instead of always falling back to uniform
    /// sampling. See this module's own doc comment ("Round 21"/"Round 24"
    /// paragraphs) for the gap this closes, and `PORTING-PLAN.md`
    /// §163.3/§164.5 for why this is caller-supplied wiring, not automatic
    /// resolution: **nothing in this crate picks a solver by name.** A
    /// caller wanting one must construct it themselves, e.g. from
    /// `moveit_kinematics::KINEMATICS_SOLVERS`, exactly as D4 already
    /// requires everywhere else in this workspace. `None` (every call
    /// site's behavior before this field existed) remains fully valid and
    /// keeps producing identical results.
    ///
    /// **What is verified for the `path_constraints` call site
    /// specifically:** `registry.rs`'s
    /// `path_constraints_solver_wiring_matches_the_call_site` test proves
    /// `resolve_constraint_sampler` (the function both call sites share)
    /// builds no sampler for a Cartesian-only region when this field is
    /// `None`, and a real IK-backed sampler producing constraint-satisfying
    /// draws when it is `Some(..)`. `path_constraints_end_to_end_wired_vs_unwired`
    /// additionally proves a full RRT-Connect path search through a
    /// Cartesian-constrained corridor benefits end-to-end, on a budget tight
    /// enough to be discriminating — round 24 had originally measured the
    /// *opposite* here (wiring made `solve()` worse, 0/5 vs. 5/5 at matched
    /// step size and budget), traced to `constrained_sampler::GroupConstraintSampler`'s
    /// per-attempt IK seed not being re-anchored between draws; round 25
    /// fixed that (see that type's own doc comment) and this field's
    /// end-to-end behavior for `path_constraints` now matches its
    /// already-verified behavior for a goal region. Setting this
    /// field is always safe either way — `checker` still enforces
    /// `path_constraints` on every candidate regardless of whether a
    /// sampler helped find one.
    ///
    /// # Why `Rc<RefCell<Box<dyn KinematicsSolver>>>`, not `Box`/`Arc`
    ///
    /// [`moveit_planning::PlannerManager::get_planning_context`] takes
    /// `&self`, so a manager cannot hand a `Box` to the context it builds
    /// without giving it up — and it must be able to build a context more
    /// than once. `Rc` clones a handle instead. `Arc<Mutex<..>>` would work
    /// too, at the cost of a `Send + Sync` bound
    /// [`moveit_kinematics::KinematicsSolver`] does not declare and mutex
    /// overhead for a caller this crate documents as single-threaded
    /// throughout. `RefCell`, not a bare `Rc`, because
    /// [`moveit_kinematics::KinematicsSolver::solve_with_options`] takes
    /// `&mut self`. A private `SharedKinematicsSolver` adapter (this
    /// module) is what actually crosses into `select_default_sampler`,
    /// since that call wants an owned `Box<dyn KinematicsSolver>`.
    pub solver: Option<Rc<RefCell<Box<dyn KinematicsSolver>>>>,
    /// The configurations this manager plans under — upstream's
    /// `PlannerManager::config_settings_` (`planning_interface.hpp:210`),
    /// except that it arrives as a constructor argument rather than through
    /// a setter (see
    /// `moveit_planner_registry::PlannerRegistration::construct`).
    ///
    /// Consulted once per query, in
    /// [`RrtConnectManager::get_planning_context`], through
    /// [`moveit_planning::configuration_for`] — not once at construction —
    /// because which entry governs is a function of the request's group and
    /// `planner_id`, exactly as upstream's own lookup is
    /// (`planning_context_manager.cpp:504-526`). Empty is the ordinary
    /// case and means every query runs on the typed fields above.
    pub configurations: PlannerConfigurationMap,
}

impl Default for RrtConnectManager {
    /// The configuration a query runs under when no
    /// [`moveit_planning::PlannerConfigurationSettings`] governs it — the
    /// floor [`RrtConnectManager::configurations`] overlays, and what
    /// `moveit_planner_registry::PLANNER_MANAGERS`' `"rrt_connect"` entry
    /// constructs from an empty map.
    ///
    /// The values are this repository's own measured ones, not an upstream
    /// citation: OMPL's `RRTConnect` defaults live in the OMPL library,
    /// which is an external dependency of moveit2 and is not present in the
    /// pinned checkout this port reads, so there is nothing upstream here
    /// to quote. `step_size: 0.5`/`goal_bias: 0.05`/`Iterations(20_000)`/
    /// `nn_degree: 8` with `resolution: 0.05` are the parameters
    /// `end_to_end_solve_on_panda_arm_reaches_the_requested_goal` has been
    /// solving panda_arm with since this planner landed; `seed: 0` is the
    /// only value a fixed default can honestly take.
    fn default() -> Self {
        Self {
            resolution: 0.05,
            seed: 0,
            params: RrtConnectParams {
                step_size: 0.5,
                goal_bias: 0.05,
                termination: Termination::Iterations(20_000),
                nn_degree: 8,
            },
            solver: None,
            configurations: PlannerConfigurationMap::new(),
        }
    }
}

impl RrtConnectManager {
    /// The manager `moveit_planner_registry::PlannerRegistration::construct`
    /// builds: [`RrtConnectManager::default`]'s tuning, planning under
    /// `configs`.
    pub fn with_planner_configurations(configs: &PlannerConfigurationMap) -> Self {
        Self {
            configurations: configs.clone(),
            ..Self::default()
        }
    }

    /// The parameters one query runs under: [`RrtConnectManager::params`],
    /// with [`RANGE_KEY`] applied from whichever entry of
    /// [`RrtConnectManager::configurations`] governs `request`.
    ///
    /// Upstream's equivalent is `planner->params().setParams(spec.config_,
    /// true)` (`planning_context_manager.cpp:213`), which hands the selected
    /// entry's whole key/value map to OMPL's own parameter system. This port
    /// binds the one key it has a parameter for and ignores the rest, which
    /// is what upstream states this map's contract to be
    /// (`planning_interface.hpp:54`, "Settings with unknown keys are
    /// ignored").
    ///
    /// A value that does not parse as an `f64`, or that
    /// [`crate::rrt_connect::RrtConnectParams`] would reject, is ignored the
    /// same way an unknown key is. Two reasons, and the second is the load
    /// bearing one: `range: 0.0` is upstream's own spelling of "let the
    /// planner pick" (`moveit_configs_utils/default_configs/ompl_defaults.yaml:40`,
    /// "if 0.0, set on setup()"), and `RrtConnectParams::assert_valid`
    /// *panics* on a non-positive step size — a value that arrived over
    /// `/set_planner_params` must not be able to take the node down.
    fn params_for(&self, request: &PlanningRequest) -> RrtConnectParams {
        let mut params = self.params.clone();
        let settings = moveit_planning::configuration_for(
            &self.configurations,
            &request.group_name,
            &request.planner_id,
        );
        if let Some(range) = settings
            .and_then(|settings| settings.config.get(RANGE_KEY))
            .and_then(|value| value.parse::<f64>().ok())
            && range.is_finite()
            && range > 0.0
        {
            params.step_size = range;
        }
        params
    }
}

/// The one [`moveit_planning::PlannerConfigurationSettings::config`] key this manager reads:
/// OMPL's `range`, "Max motion added to tree"
/// (`moveit_configs_utils/default_configs/ompl_defaults.yaml:38-40`, the
/// `RRTConnect` entry of the configuration file upstream ships as its
/// default), which is
/// [`crate::rrt_connect::RrtConnectParams::step_size`]'s quantity exactly —
/// "maximum distance a single `extend` step advances a tree toward its
/// target".
///
/// The name is upstream's and not this port's invention: it is the key
/// `ompl_planning.yaml` carries, it lands in
/// `PlannerConfigurationSettings::config` unaltered, and it reaches the
/// planner through `planner->params().setParams(spec.config_, true)`
/// (`planning_context_manager.cpp:213`). What is *not* citable from the
/// pinned checkout is OMPL's own declaration of it, since OMPL is an
/// external dependency of moveit2 and is not in that checkout — the same
/// limit [`RrtConnectManager::default`] records for the default values.
///
/// It is also the only key: RRTConnect's other OMPL parameter
/// (`intermediate_states`) has no counterpart here, and `goal_bias` —
/// which this port's RRT-Connect does have — is an `RRT`/`EST` key
/// upstream, not an `RRTConnect` one: the `RRT` block carries it
/// (`moveit_configs_utils/default_configs/ompl_defaults.yaml:34-37`) and
/// the `RRTConnect` block three lines below lists `type` and `range`
/// alone. Binding it would be inventing a name upstream does not use for
/// this planner.
pub const RANGE_KEY: &str = "range";

impl std::fmt::Debug for RrtConnectManager {
    /// Manual, not derived: [`moveit_kinematics::KinematicsSolver`] has no
    /// `Debug` bound (nothing here needs one), so `solver` cannot go
    /// through `#[derive(Debug)]` — printed as presence only.
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("RrtConnectManager")
            .field("resolution", &self.resolution)
            .field("seed", &self.seed)
            .field("params", &self.params)
            .field(
                "solver",
                &self.solver.as_ref().map(|_| "Box<dyn KinematicsSolver>"),
            )
            // Printed even though it is the longest field: it is the half of
            // this manager's tuning that arrives at runtime, so a `Debug`
            // that omitted it would show two managers planning differently
            // as identical.
            .field("configurations", &self.configurations)
            .finish()
    }
}

impl PlannerManager for RrtConnectManager {
    fn name(&self) -> &'static str {
        "rrt_connect"
    }

    fn get_planning_context<'a, 'm>(
        &self,
        scene: &'a mut PlanningScene<'m>,
        env: &'a ParryCollisionEnv,
        request: &PlanningRequest,
    ) -> Result<Box<dyn PlanningContext<'m> + 'a>, moveit_planning::PlanError> {
        let space = JointModelGroupSpace::new(scene.robot_model(), &request.group_name)
            .map_err(|err| Box::new(PlanError::Sbp(err)) as moveit_planning::PlanError)?;
        Ok(Box::new(RrtConnectContext {
            scene,
            env,
            space,
            // Cloned, matching upstream's `request_ = request`
            // (`planning_interface.hpp:114,142`): the context owns the
            // query it was built for, so a caller mutating its own request
            // afterwards cannot change what this context solves.
            request: request.clone(),
            resolution: self.resolution,
            seed: self.seed,
            params: self.params_for(request),
            solver: self.solver.clone(),
        }))
    }
}

struct RrtConnectContext<'a, 'm> {
    scene: &'a mut PlanningScene<'m>,
    env: &'a ParryCollisionEnv,
    space: JointModelGroupSpace,
    request: PlanningRequest,
    resolution: f64,
    seed: u64,
    params: RrtConnectParams,
    solver: Option<Rc<RefCell<Box<dyn KinematicsSolver>>>>,
}

/// Forwards to a solver borrowed through `Rc<RefCell<Box<dyn
/// KinematicsSolver>>>` so more than one owner can hold a
/// [`KinematicsSolver`]-shaped handle onto the *same* underlying solver
/// instance — see [`RrtConnectManager::solver`]'s own doc comment for why
/// [`RrtConnectContext::solve`] needs this.
///
/// The four `&str`/`&[String]`-returning accessors
/// ([`KinematicsSolver::group_name`], [`KinematicsSolver::joint_names`],
/// [`KinematicsSolver::base_frame`], [`KinematicsSolver::tip_frame`]) are
/// cached as owned `String`/`Vec<String>` at construction time rather than
/// forwarded live through `RefCell::borrow()`: a borrow guard is a temporary
/// that cannot outlive the method call producing it, so `&str` borrowed
/// through one cannot satisfy `fn group_name(&self) -> &str`'s `&self`
/// lifetime (`E0515`). [`KinematicsSolver::solve_with_options`] and
/// [`KinematicsSolver::tip_frames`] forward live instead, because both return
/// owned values with no such lifetime problem — and `tip_frames` *must*
/// forward rather than inherit its provided default, which would answer
/// `[self.tip_frame()]` and so report one tip for a wrapped solver that has
/// several.
struct SharedKinematicsSolver {
    inner: Rc<RefCell<Box<dyn KinematicsSolver>>>,
    group_name: String,
    joint_names: Vec<String>,
    base_frame: String,
    tip_frame: String,
}

impl SharedKinematicsSolver {
    fn new(inner: Rc<RefCell<Box<dyn KinematicsSolver>>>) -> Self {
        let (group_name, joint_names, base_frame, tip_frame) = {
            let solver = inner.borrow();
            (
                solver.group_name().to_string(),
                solver.joint_names().to_vec(),
                solver.base_frame().to_string(),
                solver.tip_frame().to_string(),
            )
        };
        Self {
            inner,
            group_name,
            joint_names,
            base_frame,
            tip_frame,
        }
    }
}

impl KinematicsSolver for SharedKinematicsSolver {
    fn group_name(&self) -> &str {
        &self.group_name
    }

    fn joint_names(&self) -> &[String] {
        &self.joint_names
    }

    fn base_frame(&self) -> &str {
        &self.base_frame
    }

    fn tip_frame(&self) -> &str {
        &self.tip_frame
    }

    fn tip_frames(&self) -> Vec<String> {
        self.inner.borrow().tip_frames()
    }

    fn solve_with_options(
        &mut self,
        seed: &[f64],
        target: &Isometry3,
        options: &mut SolveOptions,
    ) -> Option<Vec<f64>> {
        self.inner
            .borrow_mut()
            .solve_with_options(seed, target, options)
    }
}

/// The `select_default_sampler` call both the `path_constraints` and the
/// goal branches of [`RrtConnectContext::solve`] make
/// — factored out to one function so this round's boundary test
/// (`path_constraints_solver_wiring_matches_the_call_site`, below) exercises
/// the exact code `solve()` runs at the `path_constraints` call site,
/// rather than a hand-reconstructed copy of it that could silently drift
/// out of sync.
///
/// Mirrors `ompl_interface::ModelBasedPlanningContext::allocPathConstrainedSampler`/
/// `allocGoalSampler` (`model_based_planning_context.cpp`), which build
/// their `ConstraintSamplerPtr` the same way for both path and goal
/// constraints. `select_default_sampler`'s only `Err` is an unresolvable
/// name inside `subgroup_solvers` (`constraint_sampler_manager.rs:262`) —
/// structurally unreachable here since `subgroup_solvers` is always empty
/// at both call sites. `Ok(None)` (no sampler could be built — e.g. no
/// joint constraint and no solver was supplied) is not an error: it means
/// this query falls back to plain uniform sampling for that region,
/// exactly as if the constraints were absent from the sampler's point of
/// view. For `path_constraints`, correctness does not depend on a sampler
/// existing either way — `checker` in `solve()` still enforces
/// `path_constraints` on every candidate regardless of whether a sampler
/// was available to help find one.
fn resolve_constraint_sampler(
    model: &RobotModel,
    group_name: &str,
    constraints: &[Constraint],
    shared_solver: Option<Rc<RefCell<Box<dyn KinematicsSolver>>>>,
) -> Option<Box<dyn ConstraintSampler>> {
    select_default_sampler(
        model,
        group_name,
        constraints,
        shared_solver
            .map(|inner| Box::new(SharedKinematicsSolver::new(inner)) as Box<dyn KinematicsSolver>),
        vec![],
        DEFAULT_MAX_STATE_SAMPLING_ATTEMPTS,
    )
    .expect(
        "select_default_sampler's only Err is an unresolvable subgroup_solvers name; \
         subgroup_solvers is always empty here, so that path can never be taken",
    )
}

impl<'m> PlanningContext<'m> for RrtConnectContext<'_, 'm> {
    fn solve(&mut self) -> Result<PlanningResponse<'m>, moveit_planning::PlanError> {
        self.solve_inner()
            .map_err(|err| Box::new(err) as moveit_planning::PlanError)
    }
}

impl<'m> RrtConnectContext<'_, 'm> {
    /// [`PlanningContext::solve`]'s body, before the concrete
    /// [`PlanError`] is boxed into the opaque
    /// [`moveit_planning::PlanError`] the trait declares. Split out so this
    /// crate's own tests can assert on the concrete variant — a boxed
    /// `dyn Error` can only be matched by downcast, which turns "the goal
    /// could not be sampled" and "the search ran out of iterations" into
    /// the same assertion.
    fn solve_inner(&mut self) -> Result<PlanningResponse<'m>, PlanError> {
        // Read out before `checker` takes `&mut *self.scene` below.
        // `PlanningScene::robot_model` returns `&'m RobotModel` — the
        // model outlives the scene borrow, so one read up here serves
        // every later use without re-borrowing the scene.
        let model = self.scene.robot_model();
        let start = self.space.read_robot_state(self.scene.current_state());
        let template = self.scene.current_state().clone();
        let start_state = template.clone();

        // Upstream `ModelBasedPlanningContext::setGoalConstraints` drops
        // empty goal sets (`model_based_planning_context.cpp:679-683`) and
        // fails with `INVALID_GOAL_CONSTRAINTS` only if none survives
        // (`:690-694`). Done here, before any sampler is built, so an
        // all-empty request costs nothing.
        let goal_sets: Vec<&KinematicConstraintSet> = self
            .request
            .goal_constraints
            .iter()
            .filter(|set| !set.constraints().is_empty())
            .collect();
        if goal_sets.is_empty() {
            return Err(PlanError::NoGoalConstraints);
        }

        let constraint_sampler = self
            .request
            .path_constraints
            .as_ref()
            .and_then(|constraints| {
                resolve_constraint_sampler(
                    model,
                    &self.request.group_name,
                    constraints.constraints(),
                    self.solver.clone(),
                )
            });
        let path_sampler = constraint_sampler
            .as_deref()
            .map(|sampler| GroupConstraintSampler::new(&self.space, sampler, template.clone()));

        // One sampler per goal set, all built up front — upstream's
        // `constructGoal` does the same (`model_based_planning_context.cpp:519-549`:
        // one `ConstrainedGoalSampler` per set, then a
        // `GoalSampleableRegionMux` over them) rather than deferring
        // construction until a set is reached.
        let goal_samplers: Vec<Option<Box<dyn ConstraintSampler>>> = goal_sets
            .iter()
            .map(|goal_constraints| {
                resolve_constraint_sampler(
                    model,
                    &self.request.group_name,
                    goal_constraints.constraints(),
                    self.solver.clone(),
                )
            })
            .collect();

        let checker = PlanningSceneValidityChecker::new(
            &mut *self.scene,
            self.env,
            CollisionRequest::default(),
            self.request.path_constraints.as_ref(),
            &self.space,
        );
        let motion_validator = DiscreteMotionValidator::new(&checker, self.resolution);
        let mut rng = ChaCha8Rng::seed_from_u64(self.seed);

        // Upstream wraps one `ConstrainedGoalSampler` per goal set in a
        // `GoalSampleableRegionMux` and round-robins `sampleGoal` over them
        // (`detail/goal_union.cpp:83-95`). This port resolves a goal to one
        // concrete state rather than a lazily-grown region (see
        // `crate::goal_sampler::sample_goal`'s own doc), so the same
        // any-of rule reduces to: try each set in declaration order, take
        // the first that yields an accepted state. Order is the request's,
        // never the linker's or a hash map's.
        let goal = goal_sets
            .iter()
            .zip(&goal_samplers)
            .find_map(|(goal_constraints, sampler)| {
                crate::goal_sampler::sample_goal(
                    &self.space,
                    &checker,
                    goal_constraints,
                    &template,
                    sampler.as_deref(),
                    &mut rng,
                    DEFAULT_MAX_GOAL_SAMPLING_ATTEMPTS,
                )
            })
            .ok_or(PlanError::NoGoalSample)?;

        let path = rrt_connect(
            &self.space,
            &checker,
            &motion_validator,
            start,
            goal,
            Sampler {
                rng: &mut rng,
                constrained_sampler: path_sampler
                    .as_ref()
                    .map(|sampler| sampler as &dyn ConstrainedStateSampler<JointModelGroupSpace>),
            },
            &self.params,
        )?;

        // Zero duration per waypoint: RRT-Connect produces a geometric
        // path, not a timed trajectory, exactly as upstream's OMPL context
        // does — timing is what `moveit_planning`'s
        // `AddTimeOptimalParameterization`/`AddRuckigTrajectorySmoothing`
        // response adapters exist to compute afterwards.
        let mut trajectory = RobotTrajectory::for_group_name(model, &self.request.group_name)?;
        for state in path {
            let mut robot_state = template.clone();
            self.space.write_robot_state(&state, &mut robot_state);
            trajectory.add_suffix_way_point(robot_state, 0.0)?;
        }

        Ok(PlanningResponse {
            trajectory,
            planner_id: "rrt_connect".to_string(),
            start_state,
        })
    }
}

#[linkme::distributed_slice(PLANNER_MANAGERS)]
static RRT_CONNECT: PlannerRegistration = PlannerRegistration {
    name: "rrt_connect",
    construct: |configs| Box::new(RrtConnectManager::with_planner_configurations(configs)),
};

#[cfg(test)]
mod tests {
    use std::fs;
    use std::sync::Arc;

    use moveit_collision::LinkPaddingScale;
    use moveit_constraints::utils::construct_goal_joint_constraints;
    use moveit_geometry::{Cuboid, Isometry3, Shape};
    use moveit_model::{MeshSearchPaths, RobotModel};
    use moveit_srdf::SrdfModel;
    use moveit_state::RobotState;
    use rand::RngExt;

    use super::*;
    use crate::compound::CompoundValue;
    use crate::space::StateSpace;

    /// The tolerance every state goal below is built with.
    ///
    /// Zero, so these tests can go on asserting what they asserted while
    /// the goal was a `Goal::State`: the state that comes back *is* the
    /// state that was asked for. `crate::goal_sampler::sample_goal`
    /// resolves the set by drawing from it, and
    /// `moveit_constraints::JointConstraintSampler`'s draw over a
    /// zero-width window is the window's one point — see
    /// `construct_goal_joint_constraints`' own doc for the arithmetic and
    /// the measurement, and `moveit-constraints`'
    /// `tests/sampler.rs::a_zero_tolerance_goal_set_resolves_to_its_own_state`
    /// for the gate that holds it.
    const STATE_GOAL_TOLERANCE: f64 = 0.0;

    fn load_panda() -> (RobotModel, SrdfModel) {
        let root = concat!(env!("CARGO_MANIFEST_DIR"), "/../../fixtures");
        let urdf_xml = fs::read_to_string(format!("{root}/panda.urdf")).unwrap();
        let urdf = urdf_rs::read_from_string(&urdf_xml).expect("fixture URDF must parse");
        let srdf = SrdfModel::parse_file(format!("{root}/panda.srdf")).unwrap();
        let model =
            RobotModel::from_urdf_and_srdf(&urdf, &urdf_xml, &srdf, &MeshSearchPaths::none())
                .expect("fixture model must build");
        (model, srdf)
    }

    /// The goal these tests used to write as `Goal::State(value)`, in the
    /// shape upstream expresses a concrete-state goal:
    /// `constructGoalConstraints(state, jmg, tolerance)`
    /// (`kinematic_constraints/utils.hpp:99`) — one
    /// [`moveit_constraints::JointConstraint`] per group variable at that
    /// state's position, ported as
    /// [`moveit_constraints::utils::construct_goal_joint_constraints`].
    fn state_goal(
        model: &RobotModel,
        space: &JointModelGroupSpace,
        template: &RobotState<'_>,
        group_name: &str,
        value: &[CompoundValue],
    ) -> KinematicConstraintSet {
        let mut state = template.clone();
        space.write_robot_state(&value.to_vec(), &mut state);
        let posed = state.update();
        construct_goal_joint_constraints(
            model,
            &posed,
            group_name,
            STATE_GOAL_TOLERANCE,
            STATE_GOAL_TOLERANCE,
        )
        .expect("the group is real and every variable of it is set")
    }

    /// The tuning `RrtConnectManager` carries for the panda_arm queries
    /// below — [`RrtConnectManager::default`] with `seed` substituted, since
    /// several tests need distinct draws from one another.
    fn default_manager(seed: u64) -> RrtConnectManager {
        RrtConnectManager {
            seed,
            ..RrtConnectManager::default()
        }
    }

    /// A request with only the fields these tests vary; the rest are
    /// `moveit_planning::PlanningRequest`'s own defaults, which this
    /// planner does not read (workspace bounds, scaling factors,
    /// trajectory constraints).
    fn request(group_name: &str, goal: KinematicConstraintSet) -> PlanningRequest {
        PlanningRequest {
            group_name: group_name.to_string(),
            goal_constraints: vec![goal],
            ..PlanningRequest::default()
        }
    }

    #[test]
    fn rrt_connect_is_findable_by_name_in_the_registry() {
        let registration = PLANNER_MANAGERS
            .iter()
            .find(|r| r.name == "rrt_connect")
            .expect("RrtConnectManager must be registered under \"rrt_connect\"");
        let manager = (registration.construct)(&PlannerConfigurationMap::new());
        assert_eq!(manager.name(), "rrt_connect");
    }

    /// One `range` configuration, keyed the way `/set_planner_params`
    /// writes one for a group (`configuration_name("panda_arm",
    /// "RRTConnect")`).
    fn range_configuration(range: &str) -> PlannerConfigurationMap {
        let name = moveit_planning::configuration_name("panda_arm", "RRTConnect");
        let settings = moveit_planning::PlannerConfigurationSettings {
            group: "panda_arm".to_string(),
            name: name.clone(),
            config: [(RANGE_KEY.to_string(), range.to_string())]
                .into_iter()
                .collect(),
        };
        [(name, settings)].into_iter().collect()
    }

    /// The seven `panda_arm` joints, in the order every waypoint below is
    /// read in.
    const PANDA_ARM_JOINT_NAMES: [&str; 7] = [
        "panda_joint1",
        "panda_joint2",
        "panda_joint3",
        "panda_joint4",
        "panda_joint5",
        "panda_joint6",
        "panda_joint7",
    ];

    /// A concrete-state `panda_arm` goal at `panda_joint1 == position`,
    /// every other joint at its default -- the same goal shape
    /// `end_to_end_solve_on_panda_arm_reaches_the_requested_goal` uses.
    fn panda_goal_at(model: &RobotModel, position: f64) -> KinematicConstraintSet {
        let mut goal_state = RobotState::new(model);
        goal_state.set_to_default_values();
        goal_state
            .set_variable_position("panda_joint1", position)
            .expect("panda_joint1 is a panda_arm joint");
        let posed = goal_state.update();
        construct_goal_joint_constraints(
            model,
            &posed,
            "panda_arm",
            STATE_GOAL_TOLERANCE,
            STATE_GOAL_TOLERANCE,
        )
        .expect("panda_arm is real and every variable of it is set")
    }

    fn planner_request(planner_id: &str, goal: KinematicConstraintSet) -> PlanningRequest {
        PlanningRequest {
            planner_id: planner_id.to_string(),
            ..request("panda_arm", goal)
        }
    }

    /// Plans `request` through whatever `construct` builds from `configs`
    /// and returns the trajectory's waypoints, joint by joint.
    ///
    /// Goes through `PLANNER_MANAGERS` rather than through
    /// `RrtConnectManager` directly: the claim under test is that a
    /// configuration handed to the *registry* reaches the planner, and a
    /// direct construction would prove only that the field is read.
    fn waypoints_through_the_registry(
        model: &RobotModel,
        srdf: &SrdfModel,
        configs: &PlannerConfigurationMap,
        request: &PlanningRequest,
    ) -> Vec<Vec<f64>> {
        let registration = PLANNER_MANAGERS
            .iter()
            .find(|r| r.name == "rrt_connect")
            .expect("RrtConnectManager must be registered under \"rrt_connect\"");
        let manager = (registration.construct)(configs);
        let mut scene = PlanningScene::new(model, srdf);
        let env = ParryCollisionEnv::default();
        let mut context = manager
            .get_planning_context(&mut scene, &env, request)
            .expect("panda_arm is a real group");
        let response = context.solve().expect("an empty-world query is solvable");
        (0..response.trajectory.way_point_count())
            .map(|i| {
                let state = response
                    .trajectory
                    .way_point(i)
                    .expect("the index is below the count");
                PANDA_ARM_JOINT_NAMES
                    .iter()
                    .map(|name| {
                        state
                            .variable_position(name)
                            .expect("every panda_arm joint is in the model")
                    })
                    .collect()
            })
            .collect()
    }

    /// The round's success criterion (PORTING-PLAN.md §285): a
    /// configuration written into the map the registry constructs from
    /// changes the plan that comes out.
    ///
    /// `range` is RRT-Connect's per-`extend` cap, so shortening it to a
    /// tenth of `RrtConnectManager::default`'s `0.5` forces the same search
    /// to advance in smaller steps from the same seed. The comparison is
    /// against the *empty* map through the same call, so what separates the
    /// two runs is the configuration and nothing else — same registration,
    /// same seed, same scene, same request.
    #[test]
    fn a_range_configuration_reaches_the_registry_planner_and_changes_the_plan() {
        let (model, srdf) = load_panda();
        let request = planner_request("RRTConnect", panda_goal_at(&model, 0.4));

        let unconfigured = waypoints_through_the_registry(
            &model,
            &srdf,
            &PlannerConfigurationMap::new(),
            &request,
        );
        let configured =
            waypoints_through_the_registry(&model, &srdf, &range_configuration("0.05"), &request);

        assert_ne!(
            unconfigured, configured,
            "a stored `range` must change the trajectory; identical waypoints mean the \
             configuration never reached the planner"
        );
        // Directional, not just different: `range` caps how far one
        // `extend` advances, so a tenth of the default has to cross the
        // same 0.4 rad in more of them. A configuration that reached the
        // planner but landed on some *other* field (the seed, say) would
        // satisfy the `assert_ne!` above and fail here. Measured: 3
        // waypoints unconfigured, 5 configured.
        assert!(
            configured.len() > unconfigured.len(),
            "a smaller `range` must produce a finer path, got {} waypoint(s) configured \
             against {} unconfigured",
            configured.len(),
            unconfigured.len()
        );
        // Both still solve the query they were asked to solve -- otherwise
        // "different" could be satisfied by a configuration that merely
        // broke planning.
        for (label, waypoints) in [("unconfigured", &unconfigured), ("configured", &configured)] {
            // Index 0 of `PANDA_ARM_JOINT_NAMES`, the joint the goal moves.
            let reached = waypoints.last().expect("a solved plan has waypoints")[0];
            assert!(
                (reached - 0.4).abs() < 1e-6,
                "the {label} plan must still end inside the goal region, got {reached}"
            );
        }
    }

    /// The same configuration, keyed for a group this request does not name:
    /// `moveit_planning::configuration_for` must not find it, so the plan is
    /// the unconfigured one. Without this, the test above would pass just as
    /// well against a manager that applied *any* entry in the map regardless
    /// of which query it was written for.
    #[test]
    fn a_configuration_for_another_group_leaves_the_plan_alone() {
        let (model, srdf) = load_panda();
        let request = planner_request("RRTConnect", panda_goal_at(&model, 0.4));
        let mut elsewhere = range_configuration("0.05");
        let mut settings = elsewhere
            .remove("panda_arm[RRTConnect]")
            .expect("range_configuration keys it for panda_arm");
        settings.group = "hand".to_string();
        settings.name = moveit_planning::configuration_name("hand", "RRTConnect");
        elsewhere.insert(settings.name.clone(), settings);

        assert_eq!(
            waypoints_through_the_registry(&model, &srdf, &elsewhere, &request),
            waypoints_through_the_registry(
                &model,
                &srdf,
                &PlannerConfigurationMap::new(),
                &request
            ),
            "a configuration written for another group must not govern this query"
        );
    }

    /// `range` values that must be ignored rather than applied, checked
    /// through `params_for` because a rejection is invisible in a
    /// trajectory (it produces the *same* plan as the empty map, which is
    /// also what a wholly broken lookup produces).
    ///
    /// The non-positive cases are the load-bearing ones:
    /// `RrtConnectParams::assert_valid` panics on them, so applying one
    /// would turn a `/set_planner_params` call into a node crash.
    #[test]
    fn a_range_that_rrt_connect_cannot_use_is_ignored_rather_than_applied() {
        let default_step = RrtConnectManager::default().params.step_size;
        for value in ["0.0", "-0.1", "not-a-number", "", "inf", "NaN"] {
            let manager =
                RrtConnectManager::with_planner_configurations(&range_configuration(value));
            let request = PlanningRequest {
                planner_id: "RRTConnect".to_string(),
                group_name: "panda_arm".to_string(),
                ..PlanningRequest::default()
            };
            assert_eq!(
                manager.params_for(&request).step_size,
                default_step,
                "`range: {value}` must leave the compiled-in step size alone"
            );
        }
        // And the accepted case, so the loop above is not passing because
        // `params_for` ignores every value it is given.
        let manager = RrtConnectManager::with_planner_configurations(&range_configuration("0.05"));
        let request = PlanningRequest {
            planner_id: "RRTConnect".to_string(),
            group_name: "panda_arm".to_string(),
            ..PlanningRequest::default()
        };
        assert_eq!(manager.params_for(&request).step_size, 0.05);
    }

    #[test]
    fn unknown_group_is_rejected_before_any_search_runs() {
        let (model, srdf) = load_panda();
        let mut scene = PlanningScene::new(&model, &srdf);
        let env = ParryCollisionEnv::default();
        let manager = default_manager(0);

        let request = request("not_a_real_group", KinematicConstraintSet::new());

        let Err(err) = manager.get_planning_context(&mut scene, &env, &request) else {
            panic!("a group the RobotModel does not have must be rejected");
        };
        // `moveit_planning::PlanError` is `Box<dyn Error>` by design, so the
        // concrete variant is only reachable by downcast -- doing it here
        // rather than asserting on the message keeps this pinned to
        // `SbpError::UnknownGroup` specifically, not to any error that
        // happens to mention the group name.
        let concrete = err
            .downcast_ref::<PlanError>()
            .expect("this crate boxes its own PlanError");
        assert!(
            matches!(concrete, PlanError::Sbp(SbpError::UnknownGroup { .. })),
            "expected PlanError::Sbp(UnknownGroup), got {concrete:?}"
        );
    }

    /// The goal-side boundary [`PlanError::NoGoalConstraints`] marks:
    /// upstream drops empty goal sets one at a time
    /// (`model_based_planning_context.cpp:679-683`) and only returns
    /// `INVALID_GOAL_CONSTRAINTS` when nothing survives (`:690-694`). A
    /// request with one empty set is the smallest input that reaches that
    /// second branch, and it must not be confused with
    /// [`PlanError::NoGoalSample`] (a real region that could not be
    /// sampled).
    #[test]
    fn an_all_empty_goal_constraint_list_is_rejected_before_sampling() {
        let (model, srdf) = load_panda();
        let mut scene = PlanningScene::new(&model, &srdf);
        let env = ParryCollisionEnv::default();
        let manager = default_manager(0);

        let request = PlanningRequest {
            group_name: "panda_arm".to_string(),
            goal_constraints: vec![KinematicConstraintSet::new(), KinematicConstraintSet::new()],
            ..PlanningRequest::default()
        };

        let mut context = manager
            .get_planning_context(&mut scene, &env, &request)
            .expect("panda_arm is a real group");
        let err = context
            .solve()
            .expect_err("a goal list with nothing in it must not plan");
        let concrete = err
            .downcast_ref::<PlanError>()
            .expect("this crate boxes its own PlanError");
        assert!(
            matches!(concrete, PlanError::NoGoalConstraints),
            "expected PlanError::NoGoalConstraints, got {concrete:?}"
        );
    }

    /// The any-of rule itself
    /// (`ModelBasedPlanningContext::setGoalConstraints` +
    /// `GoalSampleableRegionMux`, `detail/goal_union.cpp:83-95`): a request
    /// whose *first* goal set cannot be satisfied must still plan, using a
    /// later one.
    ///
    /// The unsatisfiable set is two joint constraints on the *same* joint at
    /// positions further apart than their tolerances, not one constraint
    /// outside the joint's limits: `JointConstraint::new` clamps an
    /// out-of-limits position back onto the nearest bound
    /// (`joint.rs`, upstream `kinematic_constraint.cpp`'s own
    /// `configure`), so a constraint written that way is perfectly
    /// satisfiable by the time the sampler sees it. Measured, not assumed --
    /// this test was first written that way and passed by reaching
    /// `panda_joint1`'s upper bound, satisfying the set it was supposed to
    /// prove unsatisfiable. Two contradictory constraints instead make
    /// `JointConstraintSampler::new` intersect to an empty window (its
    /// `min_bound > max_bound` error), which `select_default_sampler` turns
    /// into "no sampler" and `decide()` then rejects every uniform draw.
    #[test]
    fn a_later_goal_set_is_used_when_an_earlier_one_cannot_be_satisfied() {
        let (model, srdf) = load_panda();
        let mut scene = PlanningScene::new(&model, &srdf);
        let env = ParryCollisionEnv::default();
        let manager = default_manager(1);

        let space = JointModelGroupSpace::new(&model, "panda_arm").unwrap();
        let mut rng = ChaCha8Rng::seed_from_u64(3);
        let reachable = space.sample_uniform(&mut rng);
        let template = scene.current_state().clone();

        let mut unreachable = KinematicConstraintSet::new();
        for position in [-2.0, 2.0] {
            unreachable.push(Constraint::Joint(
                moveit_constraints::JointConstraint::new(
                    &model,
                    "panda_joint1",
                    position,
                    STATE_GOAL_TOLERANCE,
                    STATE_GOAL_TOLERANCE,
                    1.0,
                )
                .unwrap(),
            ));
        }

        let request = PlanningRequest {
            group_name: "panda_arm".to_string(),
            goal_constraints: vec![
                unreachable,
                state_goal(&model, &space, &template, "panda_arm", &reachable),
            ],
            ..PlanningRequest::default()
        };

        let mut context = manager
            .get_planning_context(&mut scene, &env, &request)
            .expect("panda_arm is a real group");
        let response = context
            .solve()
            .expect("the second goal set is reachable, so the query must succeed");
        drop(context);

        let last = response
            .trajectory
            .way_point(response.trajectory.way_point_count() - 1)
            .unwrap();
        let reached = space.read_robot_state(last);
        assert_state_reaches(&reached, &reachable);
    }

    /// Every waypoint value of `reached` equals `expected`. No tolerance
    /// parameter, deliberately: a bound that is also the width the goal was
    /// built with can only ever confirm that the sampler stayed inside its
    /// own window, which is true however wide that window is. The contract
    /// a [`STATE_GOAL_TOLERANCE`]-built goal states is equality, so that is
    /// what this checks — the same assertion these tests made against a
    /// `Goal::State`, which the constraint encoding is supposed to carry
    /// unchanged.
    fn assert_state_reaches(reached: &[CompoundValue], expected: &[CompoundValue]) {
        assert_eq!(
            reached.len(),
            expected.len(),
            "the reached state must have the group's own shape"
        );
        for (index, (got, want)) in reached.iter().zip(expected).enumerate() {
            let (CompoundValue::RealVector(got), CompoundValue::RealVector(want)) = (got, want)
            else {
                panic!("panda_arm is an all-revolute group; subspace {index} is not a real vector");
            };
            assert_eq!(got.len(), want.len());
            for (got, want) in got.iter().zip(want) {
                assert_eq!(
                    got,
                    want,
                    "subspace {index}: reached {got}, goal {want} (gap {})",
                    got - want
                );
            }
        }
    }

    /// Registry mechanism plus [`JointModelGroupSpace`] integration on a
    /// real multi-joint fixture. Not a collision-composition test: panda's
    /// `<collision>` geometry is mesh-based and this workspace always loads
    /// fixtures with [`MeshSearchPaths::none`] (meshes are not vendored —
    /// see `moveit-planners-sbp::planning_scene_validity`'s own tests for
    /// the same constraint), so an empty-world panda scene has no collision
    /// shapes on either side and every state is trivially collision-valid.
    /// [`an_obstacle_blocks_the_direct_path_and_rrt_connect_routes_around_it`]
    /// below is the test that would fail if the collision composition
    /// itself were wrong.
    #[test]
    fn end_to_end_solve_on_panda_arm_reaches_the_requested_goal() {
        let (model, srdf) = load_panda();
        let mut scene = PlanningScene::new(&model, &srdf);
        let env = ParryCollisionEnv::default();
        let manager = default_manager(1);

        let space = JointModelGroupSpace::new(&model, "panda_arm").unwrap();
        let mut rng = ChaCha8Rng::seed_from_u64(3);
        let goal = space.sample_uniform(&mut rng);
        // Captured before `solve()`: `PlanningSceneValidityChecker::is_valid`
        // leaves the scene's current state at whatever it last checked (see
        // its own doc comment's `# Side effect`), so reading it back
        // afterward would not recover the actual start the query ran from.
        let expected_start = space.read_robot_state(scene.current_state());
        let template = scene.current_state().clone();

        let request = request(
            "panda_arm",
            state_goal(&model, &space, &template, "panda_arm", &goal),
        );

        let mut context = manager
            .get_planning_context(&mut scene, &env, &request)
            .expect("panda_arm is a real group");
        let response = context
            .solve()
            .expect("an empty-world panda_arm query must be solvable");
        drop(context);

        assert!(response.trajectory.way_point_count() >= 2);
        assert_eq!(
            response.planner_id, "rrt_connect",
            "a planner must name itself in its own response"
        );
        assert_eq!(
            response.trajectory.group_name(),
            "panda_arm",
            "the trajectory must carry the group it was planned for"
        );
        let start_positions = space.read_robot_state(response.trajectory.way_point(0).unwrap());
        let end_positions = space.read_robot_state(
            response
                .trajectory
                .way_point(response.trajectory.way_point_count() - 1)
                .unwrap(),
        );
        assert_state_reaches(&end_positions, &goal);
        assert_eq!(
            start_positions, expected_start,
            "the first waypoint must equal the scene's start state"
        );
        assert_eq!(
            space.read_robot_state(&response.start_state),
            expected_start,
            "PlanningResponse::start_state must record the state the query actually ran from"
        );
    }

    /// A synthetic, non-mesh scene with a real obstacle placed so that the
    /// straight line from start to goal passes through it. This is the test
    /// that fails if [`RrtConnectContext::solve`] silently ignored collision
    /// (e.g. wired a checker that always returns `true`): a checker that
    /// were broken that way would return the 2-waypoint straight-line path
    /// [`crate::rrt_connect::rrt_connect`]'s `connect` step tries first,
    /// which [`moveit_scene::PlanningScene::is_path_valid`] — re-run here
    /// independently of whatever checker the planner itself used — must
    /// then catch.
    #[test]
    fn an_obstacle_blocks_the_direct_path_and_rrt_connect_routes_around_it() {
        let urdf_xml = r#"<?xml version="1.0"?>
<robot name="floating_test">
  <link name="world"/>
  <link name="body">
    <collision>
      <geometry><box size="0.3 0.3 0.3"/></geometry>
    </collision>
  </link>
  <joint name="body_joint" type="floating">
    <parent link="world"/>
    <child link="body"/>
  </joint>
</robot>
"#;
        let srdf_xml = r#"<?xml version="1.0"?>
<robot name="floating_test">
  <group name="body_group">
    <joint name="body_joint"/>
  </group>
</robot>
"#;
        let urdf = urdf_rs::read_from_string(urdf_xml).expect("synthetic URDF must parse");
        let srdf = SrdfModel::parse_str(srdf_xml).expect("synthetic SRDF must parse");
        let model =
            RobotModel::from_urdf_and_srdf(&urdf, urdf_xml, &srdf, &MeshSearchPaths::none())
                .expect("synthetic model must build");

        let mut scene = PlanningScene::new(&model, &srdf);
        // A wide, thin wall centered between start (x = -2) and goal
        // (x = 2), blocking the direct line between them but leaving both
        // sides clear.
        scene.add_shape(
            "wall",
            Arc::new(Shape::Cuboid(Cuboid::new(0.2, 4.0, 4.0).unwrap())),
            Isometry3::identity(),
        );
        let env = ParryCollisionEnv::new(scene.world().clone(), LinkPaddingScale::default());

        scene
            .current_state_mut()
            .set_joint_transform("body_joint", &Isometry3::translation(-2.0, 0.0, 0.0))
            .unwrap();

        let space = JointModelGroupSpace::new(&model, "body_group").unwrap();
        let mut goal_state = RobotState::new(&model);
        goal_state.set_to_default_values();
        goal_state
            .set_joint_transform("body_joint", &Isometry3::translation(2.0, 0.0, 0.0))
            .unwrap();
        let goal = space.read_robot_state(&goal_state);

        let manager = RrtConnectManager {
            resolution: 0.02,
            seed: 3,
            params: RrtConnectParams {
                step_size: 0.3,
                goal_bias: 0.1,
                termination: Termination::Iterations(50_000),
                nn_degree: 8,
            },
            solver: None,
            configurations: PlannerConfigurationMap::new(),
        };
        let template = scene.current_state().clone();
        let request = request(
            "body_group",
            state_goal(&model, &space, &template, "body_group", &goal),
        );

        let mut context = manager
            .get_planning_context(&mut scene, &env, &request)
            .expect("body_group is a real group");
        let response = context
            .solve()
            .expect("a wall with clear space on both sides must be solvable");
        drop(context);

        assert!(
            response.trajectory.way_point_count() > 2,
            "a direct 2-waypoint path would cross the wall; a valid solution must detour"
        );

        let waypoints: Vec<RobotState> = response
            .trajectory
            .iter()
            .map(|(state, _)| state.clone())
            .collect();
        let validity =
            scene.is_path_valid(&env, &CollisionRequest::default(), &waypoints, None, &[]);
        assert!(
            validity.valid,
            "PlanningScene::is_path_valid must independently confirm every waypoint the planner returned: {:?}",
            validity.invalid_waypoints
        );
    }

    /// Proves [`RrtConnectContext::solve`]'s constraint-sampler wiring is
    /// load-bearing, not merely invoked: `panda_joint1` is pinned to
    /// `+/-0.005` (against its own `+/-2.9671` bound, `panda.urdf:37`), start
    /// and goal both already satisfy it, and `goal_bias: 0.0` forces every
    /// sample through the uniform-sampling branch — [`RrtConnectParams`]'s
    /// own doc comment covers what `goal_bias` biases toward; the point here
    /// is that with it disabled, the window is the only thing standing
    /// between the search and a solution.
    ///
    /// Both the window and the 20-iteration budget were picked by
    /// measurement, not derivation: a `+/-0.05` window at a 300-iteration
    /// budget (this test's first draft) let the unwired control find a path
    /// on roughly half of a 30-seed sweep — a straight line between two
    /// points already inside a convex interval constraint never leaves it,
    /// so RRT-Connect's own bias toward short, near-direct paths in an
    /// obstacle-free space finds one by chance often enough that "the
    /// control fails" was not a reliable property, only a lucky seed. The
    /// swept combination actually used below (`+/-0.005`, 20 iterations) is
    /// re-measured directly by this test's own loop over seeds `0..30`,
    /// asserted exactly (`PORTING-PLAN.md` §195: the same claim, once with
    /// an uncommitted, un-rerunnable "see this round's git history for the
    /// sweep" pointer that no longer resolves to anything, was the same
    /// defect round 24's own sweep left behind) — **30/30 unwired
    /// failures, 30/30 wired successes**.
    ///
    /// - **Unwired control**: [`rrt_connect`] called directly with
    ///   [`Sampler::unconstrained`] but the *same* constrained `checker` —
    ///   this is exactly what [`RrtConnectContext::solve`] would do if the
    ///   round 20 wiring did not exist. Must fail within the budget, every
    ///   seed.
    /// - **Wired**: the exact same query through the real
    ///   [`RrtConnectManager::get_planning_context`] -> `solve()` path,
    ///   same seed, same budget. Must succeed, every seed, and every
    ///   waypoint's `panda_joint1` must sit inside the window — proving the
    ///   registry's sampler wiring, not the checker alone, is what turns a
    ///   budget the checker-only search exhausts into one the search solves
    ///   within.
    #[test]
    fn path_constraint_sampler_is_load_bearing_not_merely_invoked() {
        use moveit_constraints::{Constraint, JointConstraint};

        use crate::rrt_connect::Sampler;

        let (model, srdf) = load_panda();
        let space = JointModelGroupSpace::new(&model, "panda_arm").unwrap();

        let mut goal_state = RobotState::new(&model);
        goal_state.set_to_default_values();
        for (name, value) in [
            ("panda_joint1", 0.0025),
            ("panda_joint2", 1.0),
            ("panda_joint3", -0.5),
            ("panda_joint4", -1.5),
            ("panda_joint5", 0.3),
            ("panda_joint6", 1.2),
            ("panda_joint7", 0.4),
        ] {
            goal_state.set_variable_position(name, value).unwrap();
        }
        let goal = space.read_robot_state(&goal_state);

        let small_budget = RrtConnectParams {
            step_size: 0.5,
            goal_bias: 0.0,
            termination: Termination::Iterations(20),
            nn_degree: 8,
        };

        for seed in 0..30u64 {
            let joint_constraint =
                JointConstraint::new(&model, "panda_joint1", 0.0, 0.005, 0.005, 1.0)
                    .expect("valid joint constraint");
            let mut path_constraints = KinematicConstraintSet::new();
            path_constraints.push(Constraint::Joint(joint_constraint));

            // Unwired control: same checker (constrained), plain uniform
            // sampling only.
            let mut control_scene = PlanningScene::new(&model, &srdf);
            let control_env = ParryCollisionEnv::default();
            let control_checker = PlanningSceneValidityChecker::new(
                &mut control_scene,
                &control_env,
                CollisionRequest::default(),
                Some(&path_constraints),
                &space,
            );
            let control_mv = DiscreteMotionValidator::new(&control_checker, 0.05);
            let mut default_state = RobotState::new(&model);
            default_state.set_to_default_values();
            let control_start = space.read_robot_state(&default_state);
            let control_result = rrt_connect(
                &space,
                &control_checker,
                &control_mv,
                control_start,
                goal.clone(),
                Sampler::unconstrained(&mut ChaCha8Rng::seed_from_u64(seed)),
                &small_budget,
            );
            assert_eq!(
                control_result,
                Err(PlanningFailure::IterationsExhausted),
                "seed {seed}: the unwired control (checker-only, no sampler) must NOT find the \
                 path the wired search below finds within the same iteration budget"
            );

            // Wired: the real registry path.
            let mut scene = PlanningScene::new(&model, &srdf);
            let env = ParryCollisionEnv::default();
            let manager = RrtConnectManager {
                resolution: 0.05,
                seed,
                params: small_budget.clone(),
                solver: None,
                configurations: PlannerConfigurationMap::new(),
            };
            let template = scene.current_state().clone();
            let request = PlanningRequest {
                path_constraints: Some(path_constraints),
                ..request(
                    "panda_arm",
                    state_goal(&model, &space, &template, "panda_arm", &goal),
                )
            };
            let mut context = manager
                .get_planning_context(&mut scene, &env, &request)
                .expect("panda_arm is a real group");
            let response = context.solve().unwrap_or_else(|e| {
                panic!(
                    "seed {seed}: the wired search must solve within the same iteration budget \
                     the unwired control above exhausts; got {e:?}"
                )
            });
            drop(context);

            for (index, (waypoint, _)) in response.trajectory.iter().enumerate() {
                let value = waypoint.variable_position("panda_joint1").unwrap();
                assert!(
                    (-0.005..=0.005).contains(&value),
                    "seed {seed}: waypoint {index}: panda_joint1 = {value} escaped the \
                     +/-0.005 constraint window"
                );
            }
        }
    }

    /// End-to-end proof that [`Goal::Constraints`] is wired through
    /// [`RrtConnectContext::solve`], not merely accepted and ignored: an
    /// empty-world panda_arm query whose goal is a tight (`+/-0.001`)
    /// `panda_joint1` window must produce a trajectory whose last waypoint
    /// sits inside that window — impossible for
    /// [`crate::goal_sampler::sample_goal`]'s uniform (unwired) fallback to
    /// hit reliably within [`DEFAULT_MAX_GOAL_SAMPLING_ATTEMPTS`] by the
    /// same measurement `goal_sampler::tests::constrained_branch_is_load_bearing_not_merely_invoked`
    /// makes at a *looser* `+/-0.01` window and a much smaller budget, so a
    /// solve that reliably succeeds here is only explained by the
    /// constrained branch actually running.
    #[test]
    fn goal_constraint_is_resolved_and_the_trajectory_ends_inside_the_goal_region() {
        use moveit_constraints::{Constraint, JointConstraint};

        let (model, srdf) = load_panda();
        let mut scene = PlanningScene::new(&model, &srdf);
        let env = ParryCollisionEnv::default();
        let manager = default_manager(2);

        let joint_constraint = JointConstraint::new(&model, "panda_joint1", 0.0, 0.001, 0.001, 1.0)
            .expect("valid joint constraint");
        let mut goal_constraints = KinematicConstraintSet::new();
        goal_constraints.push(Constraint::Joint(joint_constraint));

        let request = request("panda_arm", goal_constraints);

        let mut context = manager
            .get_planning_context(&mut scene, &env, &request)
            .expect("panda_arm is a real group");
        let response = context
            .solve()
            .expect("an empty-world panda_arm query with a satisfiable goal region must solve");
        drop(context);

        let value = response
            .trajectory
            .way_point(response.trajectory.way_point_count() - 1)
            .unwrap()
            .variable_position("panda_joint1")
            .unwrap();
        assert!(
            (-0.001..=0.001).contains(&value),
            "the trajectory's last waypoint has panda_joint1 = {value}, outside the +/-0.001 \
             goal region the request asked for"
        );
    }

    /// `PlanningRequest::solver` (`PORTING-PLAN.md` §163.3), boundary-tested
    /// against a goal `select_default_sampler` cannot build any other way: a
    /// Cartesian (position + orientation) region on `panda_link8`, which
    /// needs an IK winner (Step B) since no `JointConstraint` covers any
    /// `panda_arm` variable (Step A never fires).
    ///
    /// The target pose is the same reachable, off-default `panda_link8` FK
    /// pose `moveit-constraints`' own
    /// `constraint_sampler_manager::ik_alone_when_there_are_no_joint_constraints`
    /// measures a `NewtonRaphsonSolver` reliably reaching (0.02 position
    /// tolerance, 0.1 rad orientation tolerance) — reused rather than
    /// re-derived since that test already establishes those windows are
    /// large enough for the solver to converge and small enough to be a
    /// real constraint, not re-measuring the same fact here.
    ///
    /// What this test measures fresh, over the actual `solve()` path this
    /// crate runs (`select_default_sampler` -> `sample_goal` ->
    /// `rrt_connect`, not `select_default_sampler`/`sampler.sample()`
    /// alone): with `solver: None`, `select_default_sampler` cannot build an
    /// IK sampler (no full joint coverage, no solver), so
    /// `sample_goal`'s only remaining path is uniform sampling of
    /// `panda_arm`'s free 7-dimensional joint space, checked against this
    /// same tight Cartesian region — over 10 seeds (`0..10`), 0/10 found a
    /// satisfying sample within `DEFAULT_MAX_GOAL_SAMPLING_ATTEMPTS` (1000)
    /// attempts. This is also the regression half of the boundary: `None`
    /// is every `PlanningRequest` before this field existed, and this
    /// failure is the same outcome `Goal::Constraints` with a
    /// Cartesian-only goal always had when `solver: None` was the only
    /// value this call site could pass.
    #[test]
    fn solver_wiring_changes_whether_a_cartesian_pose_goal_is_reachable() {
        use moveit_constraints::{
            Constraint, OrientationConstraint, OrientationTolerance, PositionConstraint,
        };
        use moveit_geometry::{Sphere, Transforms, Vector3};
        use moveit_kinematics::{NewtonRaphsonSolver, SolverParams};

        const PANDA_ARM_JOINTS: [&str; 7] = [
            "panda_joint1",
            "panda_joint2",
            "panda_joint3",
            "panda_joint4",
            "panda_joint5",
            "panda_joint6",
            "panda_joint7",
        ];

        let (model, srdf) = load_panda();

        let true_values = [0.3, -0.4, 0.2, -1.9, 0.1, 1.2, 0.5];
        let mut fk_state = RobotState::new(&model);
        fk_state.set_to_default_values();
        for (name, &v) in PANDA_ARM_JOINTS.iter().zip(&true_values) {
            fk_state.set_variable_position(name, v).unwrap();
        }
        let target_pose = fk_state
            .update()
            .global_link_transform("panda_link8")
            .unwrap();

        let tf = Transforms::new("world").unwrap();
        let pc = PositionConstraint::new(
            &model,
            &tf,
            "panda_link8",
            "world",
            Vector3::zeros(),
            &[(Shape::Sphere(Sphere::new(0.02).unwrap()), target_pose)],
            1.0,
        )
        .unwrap();
        let oc = OrientationConstraint::new(
            &model,
            &tf,
            "panda_link8",
            "world",
            target_pose.rotation,
            OrientationTolerance::RotationVector {
                x: 0.1,
                y: 0.1,
                z: 0.1,
            },
            1.0,
        )
        .unwrap();

        let build_goal_constraints = || {
            let mut goal_constraints = KinematicConstraintSet::new();
            goal_constraints.push(Constraint::Position(pc.clone()));
            goal_constraints.push(Constraint::Orientation(oc.clone()));
            goal_constraints
        };

        for seed in 0..10u64 {
            let mut unwired_scene = PlanningScene::new(&model, &srdf);
            let env = ParryCollisionEnv::default();
            let unwired_manager = default_manager(seed);
            let unwired_request = request("panda_arm", build_goal_constraints());
            let mut unwired_context = unwired_manager
                .get_planning_context(&mut unwired_scene, &env, &unwired_request)
                .expect("panda_arm is a real group");
            let unwired_result = unwired_context.solve();
            let unwired_concrete = unwired_result
                .as_ref()
                .err()
                .and_then(|err| err.downcast_ref::<PlanError>());
            assert!(
                matches!(unwired_concrete, Some(PlanError::NoGoalSample)),
                "seed {seed}: solver: None must still fail to resolve a Cartesian-only goal \
                 (no full joint coverage, no solver -- select_default_sampler builds nothing, \
                 and 1000 uniform 7-DOF samples essentially never land inside a 0.02/0.1 \
                 Cartesian window), matching this call site's behaviour before \
                 RrtConnectManager::solver existed; got {unwired_concrete:?}"
            );
            drop(unwired_context);

            let mut wired_scene = PlanningScene::new(&model, &srdf);
            let solver: Box<dyn KinematicsSolver> = Box::new(
                NewtonRaphsonSolver::new(&model, "panda_arm", &SolverParams::default())
                    .expect("panda_arm is a chain"),
            );
            let wired_manager = RrtConnectManager {
                solver: Some(Rc::new(RefCell::new(solver))),
                ..default_manager(seed)
            };
            let wired_request = request("panda_arm", build_goal_constraints());
            let mut wired_context = wired_manager
                .get_planning_context(&mut wired_scene, &env, &wired_request)
                .expect("panda_arm is a real group");
            let response = wired_context
                .solve()
                .unwrap_or_else(|e| panic!("seed {seed}: solver: Some(..) must resolve the same goal the unwired control above could not; got {e:?}"));
            drop(wired_context);

            let mut last = response
                .trajectory
                .way_point(response.trajectory.way_point_count() - 1)
                .unwrap()
                .clone();
            let posed = last.update();
            assert!(
                pc.decide(&posed).satisfied,
                "seed {seed}: wired trajectory's last waypoint escaped the position tolerance"
            );
            assert!(
                oc.decide(&posed).satisfied,
                "seed {seed}: wired trajectory's last waypoint escaped the orientation tolerance"
            );
        }
    }

    /// `PlanningRequest::solver` (`PORTING-PLAN.md` §176), boundary-tested
    /// for the `path_constraints` call site specifically: through round 23
    /// this call site never read `self.request.solver` at all (it was
    /// consumed via `.take()` inside the goal branch only), so a
    /// Cartesian-only `path_constraints` region got no IK-backed sampler no
    /// matter what a caller passed. Round 24 fixed that by routing both call
    /// sites through the shared `resolve_constraint_sampler` helper above.
    ///
    /// This is deliberately **not** an end-to-end `solve()`/`rrt_connect`
    /// test, even though one now exists
    /// (`path_constraints_end_to_end_wired_vs_unwired`, below `PlanningRequest::solver`'s
    /// own uses): at the time this test was written,
    /// `crate::constrained_sampler::GroupConstraintSampler`'s per-attempt IK
    /// seed was not re-anchored between draws, which made a wired path
    /// sampler *not* reliably improve — and in the tightest measured
    /// scenario, actively worsen — full RRT-Connect success for
    /// path-constrained corridors, so a `solve()`-level test would have
    /// measured that confound, not this round's wiring change. Round 25
    /// fixed the seeding gap itself (see that type's own doc comment); this
    /// test stays boundary-scoped anyway, since it targets the wiring at
    /// `resolve_constraint_sampler` specifically and needs no IK-quality
    /// confound either way. Called with the exact arguments `solve()`'s
    /// `path_constraints` branch passes it (`group_name`,
    /// `subgroup_solvers: vec![]`, `DEFAULT_MAX_STATE_SAMPLING_ATTEMPTS`).
    ///
    /// **What this test proves:** `resolve_constraint_sampler` builds no
    /// sampler for a Cartesian-only region when `solver: None` — the red
    /// half, matching every request before `PlanningRequest::solver`
    /// existed — and builds a real IK-backed sampler whose draws satisfy
    /// both the position and orientation constraint when `solver: Some(..)`
    /// — the green half.
    ///
    /// **What this test does NOT prove:** that `solve()`'s `path_constraints`
    /// branch itself still passes `shared_solver.clone()` rather than some
    /// future edit reverting it to an unconditional `None` — this test calls
    /// `resolve_constraint_sampler` directly, not through `solve()`, so it
    /// cannot observe that call site's own argument. What closes that gap is
    /// structural, not this assertion: `solve()`'s `path_constraints` and
    /// goal branches (below) call `resolve_constraint_sampler` with the
    /// textually identical `shared_solver.clone()` pattern, and the goal
    /// branch's use of that exact pattern *is* independently verified
    /// end-to-end by `solver_wiring_changes_whether_a_cartesian_pose_goal_is_reachable`
    /// (above) — a reviewer comparing the two call sites, not an automated
    /// test, is what stands behind the path branch matching it. Nor does
    /// this test prove a full RRT-Connect path search through a
    /// Cartesian-constrained corridor succeeds end-to-end; see the seeding
    /// gap noted above for why that is a separate, open question.
    #[test]
    fn path_constraints_solver_wiring_matches_the_call_site() {
        use moveit_constraints::{OrientationConstraint, OrientationTolerance, PositionConstraint};
        use moveit_geometry::{Sphere, Transforms, Vector3};
        use moveit_kinematics::{NewtonRaphsonSolver, SolverParams};

        const PANDA_ARM_JOINTS: [&str; 7] = [
            "panda_joint1",
            "panda_joint2",
            "panda_joint3",
            "panda_joint4",
            "panda_joint5",
            "panda_joint6",
            "panda_joint7",
        ];

        let (model, _srdf) = load_panda();

        // Same target pose `solver_wiring_changes_whether_a_cartesian_pose_goal_is_reachable`
        // (above) already measures a `NewtonRaphsonSolver` reliably reaching
        // within these tolerances — reused rather than re-derived.
        let true_values = [0.3, -0.4, 0.2, -1.9, 0.1, 1.2, 0.5];
        let mut fk_state = RobotState::new(&model);
        fk_state.set_to_default_values();
        for (name, &v) in PANDA_ARM_JOINTS.iter().zip(&true_values) {
            fk_state.set_variable_position(name, v).unwrap();
        }
        let target_pose = fk_state
            .update()
            .global_link_transform("panda_link8")
            .unwrap();

        let tf = Transforms::new("world").unwrap();
        let pc = PositionConstraint::new(
            &model,
            &tf,
            "panda_link8",
            "world",
            Vector3::zeros(),
            &[(Shape::Sphere(Sphere::new(0.02).unwrap()), target_pose)],
            1.0,
        )
        .unwrap();
        let oc = OrientationConstraint::new(
            &model,
            &tf,
            "panda_link8",
            "world",
            target_pose.rotation,
            OrientationTolerance::RotationVector {
                x: 0.1,
                y: 0.1,
                z: 0.1,
            },
            1.0,
        )
        .unwrap();

        let mut path_constraints = KinematicConstraintSet::new();
        path_constraints.push(Constraint::Position(pc.clone()));
        path_constraints.push(Constraint::Orientation(oc.clone()));

        // Red half.
        let unwired =
            resolve_constraint_sampler(&model, "panda_arm", path_constraints.constraints(), None);
        assert!(
            unwired.is_none(),
            "solver: None must build no sampler for a Cartesian-only path_constraints region, \
             matching this call site's behaviour before PlanningRequest::solver existed"
        );

        // Green half.
        let solver: Box<dyn KinematicsSolver> = Box::new(
            NewtonRaphsonSolver::new(&model, "panda_arm", &SolverParams::default())
                .expect("panda_arm is a chain"),
        );
        let shared_solver = Some(Rc::new(RefCell::new(solver)));
        let wired = resolve_constraint_sampler(
            &model,
            "panda_arm",
            path_constraints.constraints(),
            shared_solver,
        )
        .expect(
            "solver: Some(..) must build a real IK-backed sampler for this Cartesian-only \
             region, the same target pose the goal-side wiring test above already measures a \
             NewtonRaphsonSolver reliably reaching",
        );

        let space = JointModelGroupSpace::new(&model, "panda_arm").unwrap();
        // `set_to_default_values()`, matching `solve()`'s own `template`
        // (`self.scene.current_state().clone()`, and
        // `PlanningScene::with_world` always calls
        // `set_to_default_values()` at construction) -- an all-zero
        // `RobotState::new` template converges the IK seed 0/50 times for
        // this target pose; the default-values seed converges reliably, the
        // same gap the goal-side wiring test above implicitly relies on by
        // going through the real `scene.current_state()`.
        let mut template = RobotState::new(&model);
        template.set_to_default_values();
        let bridge = GroupConstraintSampler::new(&space, wired.as_ref(), template);
        let mut rng = ChaCha8Rng::seed_from_u64(1);
        // `try_sample` returning `None` on a given call is an ordinary
        // "this attempt found nothing," not a failure -- `rrt_connect.rs`'s
        // own `sample_uniform` only ever allows the constrained sampler 3
        // tries before falling back to plain uniform sampling
        // (`rrt_connect.rs`'s `Sampler::sample_uniform` doc comment, mirroring
        // upstream `ConstrainedSampler::sampleUniform`'s
        // `!sampleC && !sampleC && !sampleC`). This loop retries generously
        // (50 attempts) per draw instead, since the property under test is
        // "can this sampler ever produce a compliant draw," not "does it
        // succeed within the 3-try budget a single `rrt_connect` growth step
        // allows" -- that tighter budget is a tree-growth performance
        // concern, not what this test measures.
        for i in 0..20 {
            let compound = (0..50)
                .find_map(|_| bridge.try_sample(&mut rng))
                .unwrap_or_else(|| panic!("draw {i}: IK-backed sampler found nothing in 50 tries"));
            let mut robot_state = RobotState::new(&model);
            robot_state.set_to_default_values();
            space.write_robot_state(&compound, &mut robot_state);
            let posed = robot_state.update();
            assert!(
                pc.decide(&posed).satisfied,
                "draw {i}: sample escaped the position tolerance"
            );
            assert!(
                oc.decide(&posed).satisfied,
                "draw {i}: sample escaped the orientation tolerance"
            );
        }
    }

    /// Re-measures `constrained_sampler::GroupConstraintSampler`'s
    /// tightest, most goal-region-analogous scenario (its own doc comment:
    /// 0/5 wired vs. 5/5 unwired, matched `step_size`/budget, before the
    /// persistent-`working` fix) against the *current* code: a
    /// position+orientation region on `panda_arm`'s `panda_link8`, start
    /// and goal both inside it but 0.8 rad apart (found by walking the
    /// pose's self-motion manifold -- see below), full RRT-Connect
    /// `solve()`, 5 seeds, matched `step_size`/iteration budget between
    /// wired and unwired.
    ///
    /// **Measured after the fix: unwired 3/5, wired 5/5** (`step_size:
    /// 0.03`, `goal_bias: 0.0`, `Iterations(20)`) -- the reverse of the
    /// pre-fix regression this scenario's ancestor recorded.
    ///
    /// The unwired count was 1/5 through round 29 and became 3/5 under D8,
    /// with no change to the search: D8 replaced this test's exact-state
    /// goal with a `constructGoalConstraints` set, so `solve()` now draws
    /// the goal out of the same `ChaCha8Rng` `rrt_connect` samples from,
    /// shifting every later draw by however many the goal sampler consumed.
    /// Isolated, not assumed: giving `sample_goal` its own stream (a
    /// one-line experiment, run and reverted) restores exactly 1/5 and 5/5
    /// here and every pinned number in
    /// `path_constraints_four_scenario_wired_vs_unwired_sweep`. The shared
    /// stream is kept because it is what round 28's `Goal::Constraints`
    /// scenario already used; a second stream would be a new design choice
    /// made to hold a number still, which is backwards. This is one
    /// scenario, not the four-scenario sweep the pre-fix measurement used
    /// (that sweep was never committed as code to re-run) -- reported as
    /// what it is, a single re-measurement showing the fix reversing the
    /// specific regression it targeted, not a full re-sweep. `PORTING-PLAN.md`
    /// §187's own committed rebuild of that sweep,
    /// `path_constraints_four_scenario_wired_vs_unwired_sweep` (below),
    /// found this exact scenario still discriminating at looser budgets
    /// too (`step_size: 0.2`/`Iterations(200)`: unwired 0/5, wired 5/5) --
    /// an earlier version of this comment claimed a looser budget made
    /// *both* solve 5/5, which does not hold for the scenario actually
    /// committed here; that claim was never re-checked against this exact
    /// code before being written down, the same failure §187 records for
    /// round 24's own uncommitted sweep.
    ///
    /// The self-motion separation is found empirically, not fabricated:
    /// independent random-restart IK (tried first) converged to distant,
    /// disconnected branches 6-7 rad away even with a tight per-seed
    /// acceptance filter -- an infeasible query regardless of sampler, not
    /// a corridor. Walking instead -- repeatedly re-solving IK for the same
    /// `target_pose` from a small random nudge of the *current* point, kept
    /// only if the result stayed close to that nudge and still satisfied
    /// `pc`/`oc` -- traces the actual connected manifold. If the walk does
    /// not reach a nontrivial separation, this test reports that honestly
    /// (`assert!` on the reached distance) rather than silently falling
    /// back to a near-identical, easy pair.
    #[test]
    fn path_constraints_end_to_end_wired_vs_unwired() {
        use moveit_constraints::{
            Constraint, OrientationConstraint, OrientationTolerance, PositionConstraint,
        };
        use moveit_geometry::{Sphere, Transforms, Vector3};
        use moveit_kinematics::{NewtonRaphsonSolver, SolveOptions, SolverParams};

        const PANDA_ARM_JOINTS: [&str; 7] = [
            "panda_joint1",
            "panda_joint2",
            "panda_joint3",
            "panda_joint4",
            "panda_joint5",
            "panda_joint6",
            "panda_joint7",
        ];

        let (model, srdf) = load_panda();

        let true_values = [0.3, -0.4, 0.2, -1.9, 0.1, 1.2, 0.5];
        let mut fk_state = RobotState::new(&model);
        fk_state.set_to_default_values();
        for (name, &v) in PANDA_ARM_JOINTS.iter().zip(&true_values) {
            fk_state.set_variable_position(name, v).unwrap();
        }
        let target_pose = fk_state
            .update()
            .global_link_transform("panda_link8")
            .unwrap();

        let tf = Transforms::new("world").unwrap();
        let pc = PositionConstraint::new(
            &model,
            &tf,
            "panda_link8",
            "world",
            Vector3::zeros(),
            &[(Shape::Sphere(Sphere::new(0.008).unwrap()), target_pose)],
            1.0,
        )
        .unwrap();
        let oc = OrientationConstraint::new(
            &model,
            &tf,
            "panda_link8",
            "world",
            target_pose.rotation,
            OrientationTolerance::RotationVector {
                x: 0.03,
                y: 0.03,
                z: 0.03,
            },
            1.0,
        )
        .unwrap();

        // Trace the self-motion manifold by many small connected steps
        // instead of independent random restarts: independent restarts
        // (tried first) converged to distant, disconnected IK branches even
        // with `max_restarts: 0` and a tight per-seed acceptance filter
        // (measured: still 6-7 rad away, an infeasible query regardless of
        // sampler). Walking accumulates a large, but genuinely *connected*,
        // displacement -- each step re-solves IK from a small random nudge
        // of the *current* point (not `true_values`), accepts it only if
        // Newton-Raphson stayed close to that nudge (not a distant
        // attractor) and the result still satisfies `pc`/`oc`, then moves
        // `current` there before the next step.
        let mut solver = NewtonRaphsonSolver::new(
            &model,
            "panda_arm",
            &SolverParams {
                max_restarts: 0,
                ..SolverParams::default()
            },
        )
        .expect("panda_arm is a chain");
        let mut current = true_values;
        let mut rng = ChaCha8Rng::seed_from_u64(7);
        for _ in 0..600u32 {
            let seed: Vec<f64> = current
                .iter()
                .map(|&v| v + rng.random_range(-0.05..0.05))
                .collect();
            let mut options = SolveOptions::default();
            let Some(solution) = solver.solve_with_options(&seed, &target_pose, &mut options)
            else {
                continue;
            };
            let step_dist: f64 = seed
                .iter()
                .zip(&solution)
                .map(|(s, v)| (s - v).powi(2))
                .sum::<f64>()
                .sqrt();
            if step_dist > 0.15 {
                continue;
            }
            let mut candidate_state = RobotState::new(&model);
            candidate_state.set_to_default_values();
            for (name, &v) in PANDA_ARM_JOINTS.iter().zip(&solution) {
                candidate_state.set_variable_position(name, v).unwrap();
            }
            let posed = candidate_state.update();
            if pc.decide(&posed).satisfied && oc.decide(&posed).satisfied {
                current = solution.try_into().unwrap();
            }
        }

        let best_distance: f64 = true_values
            .iter()
            .zip(&current)
            .map(|(a, b)| (a - b).powi(2))
            .sum::<f64>()
            .sqrt();
        assert!(
            best_distance > 0.3,
            "walking panda_arm's self-motion manifold from true_values did not reach a \
             nontrivial separation ({best_distance} rad) -- this scenario cannot exercise \
             self-motion within the corridor"
        );

        let start_values = true_values;
        let goal_values = current;

        let space = JointModelGroupSpace::new(&model, "panda_arm").unwrap();

        let mut goal_state = RobotState::new(&model);
        goal_state.set_to_default_values();
        for (name, &v) in PANDA_ARM_JOINTS.iter().zip(&goal_values) {
            goal_state.set_variable_position(name, v).unwrap();
        }
        let goal = space.read_robot_state(&goal_state);

        // Matched small budget, same shape as
        // `path_constraint_sampler_is_load_bearing_not_merely_invoked`'s
        // own `small_budget` -- large enough that an unconstrained straight
        // line could plausibly succeed, small enough that a search
        // struggling to stay in a narrow corridor visibly fails within it.
        let small_budget = RrtConnectParams {
            step_size: 0.03,
            goal_bias: 0.0,
            termination: Termination::Iterations(20),
            nn_degree: 8,
        };

        let build_path_constraints = || {
            let mut path_constraints = KinematicConstraintSet::new();
            path_constraints.push(Constraint::Position(pc.clone()));
            path_constraints.push(Constraint::Orientation(oc.clone()));
            path_constraints
        };

        let set_start = |scene: &mut PlanningScene<'_>| {
            scene.current_state_mut().set_to_default_values();
            for (name, &v) in PANDA_ARM_JOINTS.iter().zip(&start_values) {
                scene
                    .current_state_mut()
                    .set_variable_position(name, v)
                    .unwrap();
            }
        };

        let mut unwired_successes = 0;
        let mut wired_successes = 0;
        for seed in 0..5u64 {
            let mut unwired_scene = PlanningScene::new(&model, &srdf);
            set_start(&mut unwired_scene);
            let env = ParryCollisionEnv::default();
            let unwired_manager = RrtConnectManager {
                resolution: 0.05,
                seed,
                params: small_budget.clone(),
                solver: None,
                configurations: PlannerConfigurationMap::new(),
            };
            let template = unwired_scene.current_state().clone();
            let unwired_request = PlanningRequest {
                path_constraints: Some(build_path_constraints()),
                ..request(
                    "panda_arm",
                    state_goal(&model, &space, &template, "panda_arm", &goal),
                )
            };
            let mut unwired_context = unwired_manager
                .get_planning_context(&mut unwired_scene, &env, &unwired_request)
                .expect("panda_arm is a real group");
            if unwired_context.solve().is_ok() {
                unwired_successes += 1;
            }
            drop(unwired_context);

            let mut wired_scene = PlanningScene::new(&model, &srdf);
            set_start(&mut wired_scene);
            let wired_solver: Box<dyn KinematicsSolver> = Box::new(
                NewtonRaphsonSolver::new(&model, "panda_arm", &SolverParams::default())
                    .expect("panda_arm is a chain"),
            );
            let wired_manager = RrtConnectManager {
                resolution: 0.05,
                seed,
                params: small_budget.clone(),
                solver: Some(Rc::new(RefCell::new(wired_solver))),
                configurations: PlannerConfigurationMap::new(),
            };
            let wired_request = PlanningRequest {
                path_constraints: Some(build_path_constraints()),
                ..request(
                    "panda_arm",
                    state_goal(&model, &space, &template, "panda_arm", &goal),
                )
            };
            let mut wired_context = wired_manager
                .get_planning_context(&mut wired_scene, &env, &wired_request)
                .expect("panda_arm is a real group");
            if wired_context.solve().is_ok() {
                wired_successes += 1;
            }
            drop(wired_context);
        }

        eprintln!(
            "path_constraints_end_to_end_wired_vs_unwired: unwired {unwired_successes}/5, \
             wired {wired_successes}/5 (self-motion distance {best_distance} rad)"
        );
        // `PORTING-PLAN.md` §195: this doc comment's own "Measured after
        // the fix: unwired 1/5, wired 5/5" was, until now, backed only by
        // `wired_successes > unwired_successes` -- weaker than the cited
        // number, and silently satisfiable by e.g. unwired 3/5. Both seed
        // counts are `ChaCha8Rng`-deterministic, so pinned exactly; this is
        // the same scenario `path_constraints_four_scenario_wired_vs_unwired_sweep`'s
        // scenario 1 now pins independently, and the two are expected to
        // agree since they share the same geometry and budget.
        assert_eq!(
            (unwired_successes, wired_successes),
            (3, 5),
            "moved off the documented unwired 3/5, wired 5/5"
        );
    }

    /// `PORTING-PLAN.md` §187: round 24 measured **wired 0/5 vs unwired
    /// 5/5** -- the opposite direction from
    /// `path_constraints_end_to_end_wired_vs_unwired`'s **unwired 1/5,
    /// wired 5/5** -- across a four-scenario sweep that was never committed
    /// as reusable code (`doc/claim-audit/moveit-planners-sbp.md`'s round-24
    /// row), so that number could not be re-run or checked against
    /// anything. This rebuilds that sweep as committed, runnable code, with
    /// four scenarios varied enough to cover more than the single
    /// `Goal::State` shape `path_constraints_end_to_end_wired_vs_unwired`
    /// uses (scenario 2's goal is `Goal::Constraints`) -- and reports each
    /// scenario's wired/unwired success counts rather than asserting a
    /// predetermined direction. Which direction each scenario shows *is*
    /// the measurement here, not a regression to guard: this test asserts
    /// only that each scenario's setup is well-formed (a reachable target,
    /// a nontrivial self-motion separation), never which of wired/unwired
    /// won.
    ///
    /// 1. **Self-motion path corridor, `Goal::State`.** Identical geometry
    ///    and budget to `path_constraints_end_to_end_wired_vs_unwired`,
    ///    included here for side-by-side comparison under the same harness.
    /// 2. **Cartesian `Goal::Constraints`, no path corridor.** The goal
    ///    itself needs IK-backed sampling to be reachable at all (mirrors
    ///    `solver_wiring_changes_whether_a_cartesian_pose_goal_is_reachable`'s
    ///    shape); no `path_constraints`.
    /// 3. **Orientation-only path corridor, free position.** No
    ///    `PositionConstraint` at all -- translation along the corridor is
    ///    entirely free, only orientation is held, modelling an
    ///    unconstrained approach axis.
    /// 4. **Budget crossover sweep.** Scenario 1's own geometry, re-run at
    ///    three budgets from tight to loose, to see where (if anywhere) a
    ///    wired/unwired gap closes.
    ///
    /// # Measured
    ///
    /// ```text
    /// scenario 1 (self-motion, concrete-state goal): unwired 3/5, wired 5/5
    /// scenario 2 (constraint-region goal):            unwired 0/5, wired 5/5
    /// scenario 3 (orientation-only corridor):         unwired 5/5, wired 5/5
    /// scenario 4, tight  (0.03/Iterations(20)):       unwired 3/5, wired 5/5
    /// scenario 4, medium (0.1 /Iterations(20)):        unwired 0/5, wired 5/5
    /// scenario 4, loose  (0.2 /Iterations(200)):       unwired 0/5, wired 5/5
    /// ```
    ///
    /// Scenarios 1 and 4-tight read 1/5 through round 29; D8's move from an
    /// exact-state goal to a `constructGoalConstraints` set put the goal
    /// draw on the same `ChaCha8Rng` the search samples from, shifting the
    /// stream. Isolated and reverted: a private goal stream restores every
    /// pre-D8 number here exactly. See
    /// `path_constraints_end_to_end_wired_vs_unwired`'s doc for why the
    /// shared stream is kept.
    ///
    /// Reproduced identically across repeated runs (deterministic seeds
    /// throughout). Three readings, none smoothed over:
    ///
    /// - Round 24's direction (wired *worse* than unwired) does not
    ///   reproduce anywhere in this sweep. Every scenario has wired >=
    ///   unwired; most have wired strictly better. Round 24's number was
    ///   either scenario-specific in a way none of these four scenarios
    ///   happen to recreate, or wrong -- this sweep cannot distinguish
    ///   those two without round 24's own scenario, which no longer exists
    ///   as re-runnable code.
    /// - Scenario 3 (orientation-only corridor) is the one scenario where
    ///   wired does *not* beat unwired -- both solve 5/5. An orientation-only
    ///   corridor with position entirely free is, per this measurement,
    ///   easy enough for uniform joint-space sampling alone that an
    ///   IK-backed sampler has nothing to add.
    /// - Scenario 4 corrects a stale claim: an earlier version of
    ///   `path_constraints_end_to_end_wired_vs_unwired`'s own doc comment
    ///   asserted that a looser budget (`step_size: 0.2`) made *both* wired
    ///   and unwired solve 5/5 for scenario 1's exact geometry -- checked
    ///   directly against that committed test with the loose budget
    ///   substituted in, both `Iterations(200)` and `Iterations(20)` in
    ///   fact measure unwired 0/5, matching this sweep. That claim was
    ///   itself an uncommitted-measurement casualty of the kind `PORTING-PLAN.md`
    ///   §187 exists to prevent; both doc comments are corrected together
    ///   in the commit that added this sweep.
    #[test]
    fn path_constraints_four_scenario_wired_vs_unwired_sweep() {
        use moveit_constraints::{
            Constraint, OrientationConstraint, OrientationTolerance, PositionConstraint,
        };
        use moveit_geometry::{Sphere, Transforms, Vector3};
        use moveit_kinematics::{NewtonRaphsonSolver, SolveOptions, SolverParams};

        const PANDA_ARM_JOINTS: [&str; 7] = [
            "panda_joint1",
            "panda_joint2",
            "panda_joint3",
            "panda_joint4",
            "panda_joint5",
            "panda_joint6",
            "panda_joint7",
        ];

        fn fk_pose(model: &RobotModel, values: &[f64]) -> Isometry3 {
            let mut state = RobotState::new(model);
            state.set_to_default_values();
            for (name, &v) in PANDA_ARM_JOINTS.iter().zip(values) {
                state.set_variable_position(name, v).unwrap();
            }
            state.update().global_link_transform("panda_link8").unwrap()
        }

        /// Runs `num_seeds` wired and `num_seeds` unwired `solve()`s for one
        /// scenario, returning `(wired_successes, unwired_successes)`.
        /// `goal`/`path_constraints` are factories rather than values so
        /// each of the `2 * num_seeds` requests gets an independently owned
        /// copy; the solver, now a `RrtConnectManager` field, is built once
        /// per manager instead.
        fn run_scenario(
            model: &RobotModel,
            srdf: &SrdfModel,
            start_values: &[f64],
            goal: impl Fn() -> KinematicConstraintSet,
            path_constraints: impl Fn() -> Option<KinematicConstraintSet>,
            budget: &RrtConnectParams,
            num_seeds: u64,
        ) -> (u32, u32) {
            let set_start = |scene: &mut PlanningScene<'_>| {
                scene.current_state_mut().set_to_default_values();
                for (name, &v) in PANDA_ARM_JOINTS.iter().zip(start_values) {
                    scene
                        .current_state_mut()
                        .set_variable_position(name, v)
                        .unwrap();
                }
            };
            let env = ParryCollisionEnv::default();
            let mut wired_successes = 0u32;
            let mut unwired_successes = 0u32;
            for seed in 0..num_seeds {
                let mut unwired_scene = PlanningScene::new(model, srdf);
                set_start(&mut unwired_scene);
                let unwired_manager = RrtConnectManager {
                    resolution: 0.05,
                    seed,
                    params: budget.clone(),
                    solver: None,
                    configurations: PlannerConfigurationMap::new(),
                };
                let unwired_request = PlanningRequest {
                    path_constraints: path_constraints(),
                    ..request("panda_arm", goal())
                };
                let mut unwired_context = unwired_manager
                    .get_planning_context(&mut unwired_scene, &env, &unwired_request)
                    .expect("panda_arm is a real group");
                if unwired_context.solve().is_ok() {
                    unwired_successes += 1;
                }
                drop(unwired_context);

                let mut wired_scene = PlanningScene::new(model, srdf);
                set_start(&mut wired_scene);
                let wired_solver: Box<dyn KinematicsSolver> = Box::new(
                    NewtonRaphsonSolver::new(model, "panda_arm", &SolverParams::default())
                        .expect("panda_arm is a chain"),
                );
                let wired_manager = RrtConnectManager {
                    resolution: 0.05,
                    seed,
                    params: budget.clone(),
                    solver: Some(Rc::new(RefCell::new(wired_solver))),
                    configurations: PlannerConfigurationMap::new(),
                };
                let wired_request = PlanningRequest {
                    path_constraints: path_constraints(),
                    ..request("panda_arm", goal())
                };
                let mut wired_context = wired_manager
                    .get_planning_context(&mut wired_scene, &env, &wired_request)
                    .expect("panda_arm is a real group");
                if wired_context.solve().is_ok() {
                    wired_successes += 1;
                }
                drop(wired_context);
            }
            (wired_successes, unwired_successes)
        }

        let (model, srdf) = load_panda();
        let tf = Transforms::new("world").unwrap();

        // --- Scenario 1: self-motion path corridor, Goal::State ---
        // Identical construction to `path_constraints_end_to_end_wired_vs_unwired`.
        let true_values = [0.3, -0.4, 0.2, -1.9, 0.1, 1.2, 0.5];
        let target_pose = fk_pose(&model, &true_values);
        let pc1 = PositionConstraint::new(
            &model,
            &tf,
            "panda_link8",
            "world",
            Vector3::zeros(),
            &[(Shape::Sphere(Sphere::new(0.008).unwrap()), target_pose)],
            1.0,
        )
        .unwrap();
        let oc1 = OrientationConstraint::new(
            &model,
            &tf,
            "panda_link8",
            "world",
            target_pose.rotation,
            OrientationTolerance::RotationVector {
                x: 0.03,
                y: 0.03,
                z: 0.03,
            },
            1.0,
        )
        .unwrap();
        let mut walk_solver = NewtonRaphsonSolver::new(
            &model,
            "panda_arm",
            &SolverParams {
                max_restarts: 0,
                ..SolverParams::default()
            },
        )
        .expect("panda_arm is a chain");
        let mut current = true_values;
        let mut walk_rng = ChaCha8Rng::seed_from_u64(7);
        for _ in 0..600u32 {
            let seed: Vec<f64> = current
                .iter()
                .map(|&v| v + walk_rng.random_range(-0.05..0.05))
                .collect();
            let mut options = SolveOptions::default();
            let Some(solution) = walk_solver.solve_with_options(&seed, &target_pose, &mut options)
            else {
                continue;
            };
            let step_dist: f64 = seed
                .iter()
                .zip(&solution)
                .map(|(s, v)| (s - v).powi(2))
                .sum::<f64>()
                .sqrt();
            if step_dist > 0.15 {
                continue;
            }
            let mut candidate_state = RobotState::new(&model);
            candidate_state.set_to_default_values();
            for (name, &v) in PANDA_ARM_JOINTS.iter().zip(&solution) {
                candidate_state.set_variable_position(name, v).unwrap();
            }
            let candidate_posed = candidate_state.update();
            if pc1.decide(&candidate_posed).satisfied && oc1.decide(&candidate_posed).satisfied {
                current = solution.try_into().unwrap();
            }
        }
        let scenario1_distance: f64 = true_values
            .iter()
            .zip(&current)
            .map(|(a, b)| (a - b).powi(2))
            .sum::<f64>()
            .sqrt();
        assert!(
            scenario1_distance > 0.3,
            "scenario 1: walking panda_arm's self-motion manifold from true_values did not \
             reach a nontrivial separation ({scenario1_distance} rad)"
        );
        let scenario1_goal_values = current;
        let scenario1_budget = RrtConnectParams {
            step_size: 0.03,
            goal_bias: 0.0,
            termination: Termination::Iterations(20),
            nn_degree: 8,
        };
        let scenario1_goal = || {
            let mut goal_state = RobotState::new(&model);
            goal_state.set_to_default_values();
            for (name, &v) in PANDA_ARM_JOINTS.iter().zip(&scenario1_goal_values) {
                goal_state.set_variable_position(name, v).unwrap();
            }
            let posed = goal_state.update();
            construct_goal_joint_constraints(
                &model,
                &posed,
                "panda_arm",
                STATE_GOAL_TOLERANCE,
                STATE_GOAL_TOLERANCE,
            )
            .expect("panda_arm is real and every variable of it is set")
        };
        let scenario1_path_constraints = || {
            let mut set = KinematicConstraintSet::new();
            set.push(Constraint::Position(pc1.clone()));
            set.push(Constraint::Orientation(oc1.clone()));
            Some(set)
        };
        let (scenario1_wired, scenario1_unwired) = run_scenario(
            &model,
            &srdf,
            &true_values,
            scenario1_goal,
            scenario1_path_constraints,
            &scenario1_budget,
            5,
        );

        // --- Scenario 2: Cartesian Goal::Constraints, no path corridor ---
        let scenario2_start = [0.0f64; 7];
        let target_pose2 = fk_pose(&model, &[0.1, -0.3, 0.4, -1.6, 0.2, 1.0, 0.7]);
        let pc2 = PositionConstraint::new(
            &model,
            &tf,
            "panda_link8",
            "world",
            Vector3::zeros(),
            &[(Shape::Sphere(Sphere::new(0.02).unwrap()), target_pose2)],
            1.0,
        )
        .unwrap();
        let oc2 = OrientationConstraint::new(
            &model,
            &tf,
            "panda_link8",
            "world",
            target_pose2.rotation,
            OrientationTolerance::RotationVector {
                x: 0.1,
                y: 0.1,
                z: 0.1,
            },
            1.0,
        )
        .unwrap();
        let scenario2_goal = || {
            let mut set = KinematicConstraintSet::new();
            set.push(Constraint::Position(pc2.clone()));
            set.push(Constraint::Orientation(oc2.clone()));
            set
        };
        let scenario2_budget = RrtConnectParams {
            step_size: 0.5,
            goal_bias: 0.05,
            termination: Termination::Iterations(2000),
            nn_degree: 8,
        };
        let (scenario2_wired, scenario2_unwired) = run_scenario(
            &model,
            &srdf,
            &scenario2_start,
            scenario2_goal,
            || None,
            &scenario2_budget,
            5,
        );

        // --- Scenario 3: orientation-only path corridor, free position ---
        let mut target_pose3 = target_pose;
        target_pose3.translation.vector.x += 0.15;
        let mut oc3_solver =
            NewtonRaphsonSolver::new(&model, "panda_arm", &SolverParams::default())
                .expect("panda_arm is a chain");
        let mut oc3_options = SolveOptions::default();
        let scenario3_goal_values = oc3_solver
            .solve_with_options(&true_values, &target_pose3, &mut oc3_options)
            .expect(
                "scenario 3: target_pose3 (target_pose shifted +0.15m along world X, same \
                 orientation) must be IK-reachable from true_values",
            );
        let oc3 = OrientationConstraint::new(
            &model,
            &tf,
            "panda_link8",
            "world",
            target_pose.rotation,
            OrientationTolerance::RotationVector {
                x: 0.05,
                y: 0.05,
                z: 0.05,
            },
            1.0,
        )
        .unwrap();
        {
            let mut goal_check_state = RobotState::new(&model);
            goal_check_state.set_to_default_values();
            for (name, &v) in PANDA_ARM_JOINTS.iter().zip(&scenario3_goal_values) {
                goal_check_state.set_variable_position(name, v).unwrap();
            }
            let posed = goal_check_state.update();
            assert!(
                oc3.decide(&posed).satisfied,
                "scenario 3: the IK-solved goal must itself satisfy the orientation-only \
                 corridor it is the target of"
            );
        }
        let scenario3_goal = || {
            let mut goal_state = RobotState::new(&model);
            goal_state.set_to_default_values();
            for (name, &v) in PANDA_ARM_JOINTS.iter().zip(&scenario3_goal_values) {
                goal_state.set_variable_position(name, v).unwrap();
            }
            let posed = goal_state.update();
            construct_goal_joint_constraints(
                &model,
                &posed,
                "panda_arm",
                STATE_GOAL_TOLERANCE,
                STATE_GOAL_TOLERANCE,
            )
            .expect("panda_arm is real and every variable of it is set")
        };
        let scenario3_path_constraints = || {
            let mut set = KinematicConstraintSet::new();
            set.push(Constraint::Orientation(oc3.clone()));
            Some(set)
        };
        let (scenario3_wired, scenario3_unwired) = run_scenario(
            &model,
            &srdf,
            &true_values,
            scenario3_goal,
            scenario3_path_constraints,
            &scenario1_budget,
            5,
        );

        // --- Scenario 4: budget crossover sweep, scenario 1's own geometry ---
        let crossover_budgets = [
            (
                "tight (0.03/Iterations(20))",
                RrtConnectParams {
                    step_size: 0.03,
                    goal_bias: 0.0,
                    termination: Termination::Iterations(20),
                    nn_degree: 8,
                },
            ),
            (
                "medium (0.1/Iterations(20))",
                RrtConnectParams {
                    step_size: 0.1,
                    goal_bias: 0.0,
                    termination: Termination::Iterations(20),
                    nn_degree: 8,
                },
            ),
            (
                "loose (0.2/Iterations(200))",
                RrtConnectParams {
                    step_size: 0.2,
                    goal_bias: 0.0,
                    termination: Termination::Iterations(200),
                    nn_degree: 8,
                },
            ),
        ];
        let scenario4_results: Vec<(&str, u32, u32)> = crossover_budgets
            .iter()
            .map(|(label, budget)| {
                let (wired, unwired) = run_scenario(
                    &model,
                    &srdf,
                    &true_values,
                    scenario1_goal,
                    scenario1_path_constraints,
                    budget,
                    5,
                );
                (*label, wired, unwired)
            })
            .collect();

        eprintln!(
            "path_constraints_four_scenario_wired_vs_unwired_sweep:\n\
             \x20 scenario 1 (self-motion, Goal::State):        unwired {scenario1_unwired}/5, wired {scenario1_wired}/5\n\
             \x20 scenario 2 (Goal::Constraints, no corridor):   unwired {scenario2_unwired}/5, wired {scenario2_wired}/5\n\
             \x20 scenario 3 (orientation-only corridor):        unwired {scenario3_unwired}/5, wired {scenario3_wired}/5\n\
             \x20 scenario 4 (budget crossover, scenario 1 geometry):"
        );
        for (label, wired, unwired) in &scenario4_results {
            eprintln!("    {label}: unwired {unwired}/5, wired {wired}/5");
        }

        // `PORTING-PLAN.md` §195: a number cited in this test's own doc
        // comment (the "# Measured" table above) or in
        // `doc/claim-audit/moveit-planners-sbp.md` is a claim, not merely an
        // observation, the moment something else depends on it -- and
        // nothing kept these six true until now (verified: routing the
        // wired branch's solver to `None` inside `run_scenario`, deleting
        // the entire effect this sweep exists to measure, left this test
        // green). All seeds are `ChaCha8Rng`-deterministic, so every number
        // is pinned exactly rather than left as an inequality: either
        // direction moving is a real change to what this sweep reports, not
        // noise to tolerate. Scenario 3's `(5, 5)` is asserted too, even
        // though wired and unwired are equal there -- that equality is
        // itself the scenario's finding (see this test's doc comment and
        // `doc/claim-audit/moveit-planners-sbp.md`), and is exactly as
        // capable of silently drifting as the other five.
        assert_eq!(
            (scenario1_unwired, scenario1_wired),
            (3, 5),
            "scenario 1 (self-motion, concrete-state goal) moved off the documented unwired 3/5, \
             wired 5/5"
        );
        assert_eq!(
            (scenario2_unwired, scenario2_wired),
            (0, 5),
            "scenario 2 (Goal::Constraints, no corridor) moved off the documented unwired 0/5, wired 5/5"
        );
        assert_eq!(
            (scenario3_unwired, scenario3_wired),
            (5, 5),
            "scenario 3 (orientation-only corridor) moved off the documented unwired 5/5, wired 5/5 tie"
        );
        let expected_scenario4 = [
            ("tight (0.03/Iterations(20))", 5u32, 3u32),
            ("medium (0.1/Iterations(20))", 5u32, 0u32),
            ("loose (0.2/Iterations(200))", 5u32, 0u32),
        ];
        assert_eq!(
            scenario4_results.len(),
            expected_scenario4.len(),
            "scenario 4's budget count changed -- zip below would silently drop the extra \
             budgets' assertions otherwise"
        );
        for ((label, wired, unwired), (expected_label, expected_wired, expected_unwired)) in
            scenario4_results.iter().zip(&expected_scenario4)
        {
            assert_eq!(
                label, expected_label,
                "scenario 4's budget ordering/labels changed"
            );
            assert_eq!(
                (*unwired, *wired),
                (*expected_unwired, *expected_wired),
                "scenario 4 {label} moved off the documented unwired {expected_unwired}/5, wired {expected_wired}/5"
            );
        }
    }

    /// `PORTING-PLAN.md` §195/§196: scenario 3's `(5, 5)` tie above is the
    /// sweep's most interesting result, and until this test existed it was
    /// only inferred from *plan* success, never measured at the sample
    /// level. Two readings were proposed, both consistent with a tie and
    /// with different consequences for what the wiring is worth: (a)
    /// uniform joint-space sampling already satisfies the orientation-only
    /// corridor often enough that an IK-backed sampler has real headroom to
    /// help elsewhere but none here, or (b) the corridor as built is close
    /// to *vacuous* for `panda_arm` at this pose -- satisfied by nearly
    /// every configuration -- so a tie says nothing about the wiring at all
    /// (§196: "the constraint was never binding" from another panel this
    /// round, same failure shape). Measuring directly, at two different
    /// granularities, shows **neither reading survives as originally
    /// posed** -- the real explanation is a third thing, also measured
    /// here rather than argued.
    ///
    /// # Measurement 1: global rate -- inconclusive, and that is itself the finding
    ///
    /// The fraction of independent uniform `panda_arm` joint-space draws
    /// ([`crate::space::StateSpace::sample_uniform`] via
    /// [`JointModelGroupSpace`], the same draw RRT-Connect's own
    /// unconstrained branch takes) that satisfy each path constraint on its
    /// own, no search involved:
    ///
    /// ```text
    /// scenario 1 (position 0.008 sphere + orientation +/-0.03 rad): 0/20,000 satisfied
    /// scenario 3 (orientation-only corridor, +/-0.05 rad, free position): 2/200,000 satisfied
    /// ```
    ///
    /// This refutes (b): a vacuous corridor satisfied by nearly every
    /// configuration would not need 200,000 draws to find 2 hits. It does
    /// *not* clearly support (a) either -- scenario 3's rate (~1e-5) is not
    /// reliably higher than scenario 1's (bounded above by roughly `3/20,000`
    /// at zero hits, the standard rule-of-thumb upper bound), so an
    /// independent global sample cannot explain why scenario 3 solves 5/5
    /// unwired while scenario 1 solves only 1/5: by this measurement the two
    /// corridors are comparably, vanishingly rare, yet the plan-level
    /// outcomes differ by 4/5. Global sample density is the wrong quantity
    /// to explain RRT-Connect's actual behavior with.
    ///
    /// # Measurement 2: local step acceptance -- the quantity that actually matches RRT-Connect
    ///
    /// [`crate::rrt_connect::rrt_connect`]'s own `extend` never draws an
    /// independent global sample and asks whether *it* satisfies the
    /// corridor; it takes a `step_size`-bounded step from an *existing tree
    /// node already inside the corridor* toward a random target, and only
    /// that bounded step needs to stay valid. This measures exactly that:
    /// starting at `true_values` (`target_pose`'s own joint configuration,
    /// so it trivially satisfies both scenarios' constraints at 0
    /// deviation, a valid interior point for either without extra IK
    /// solving), one `step_size = 0.03` (`scenario1_budget`'s own value)
    /// random joint perturbation per draw, checked against the same
    /// scenario's constraint:
    ///
    /// ```text
    /// scenario 1: 1,837/20,000 accepted (9.2%)
    /// scenario 3: 16,586/20,000 accepted (83.0%)
    /// ```
    ///
    /// This is the number that actually explains the tie. Locally, a
    /// scenario-3 step stays inside the corridor roughly 9x more often than
    /// a scenario-1 step -- position pins the corridor to a 0.008-radius
    /// ball in 3 more dimensions than orientation alone constrains, so a
    /// random `step_size`-sized move is far more likely to wander back out
    /// of scenario 1's corridor than out of scenario 3's. That is reading
    /// (a) -- uniform local exploration genuinely has more headroom in
    /// scenario 3 -- but the *global* draw rate (measurement 1) does not
    /// show it; only the *local*, already-inside-the-corridor rate does,
    /// because that is what tree growth actually samples against. 83% also
    /// refutes (b) precisely: a vacuous constraint would accept close to
    /// 100% of local steps, not 83% -- the orientation window is genuinely
    /// binding, just far less so locally than scenario 1's tighter
    /// position+orientation window. Reading (a) is correct, but the
    /// evidence for it is measurement 2, not measurement 1 -- a caution
    /// against inferring "the sampler has no headroom" from a global rate
    /// when the search itself never actually draws globally once inside a
    /// region.
    #[test]
    fn scenario3_orientation_only_corridor_sample_level_satisfaction_rate() {
        use moveit_constraints::{OrientationConstraint, OrientationTolerance, PositionConstraint};
        use moveit_geometry::{Sphere, Transforms, Vector3};
        use moveit_state::Posed;

        const PANDA_ARM_JOINTS: [&str; 7] = [
            "panda_joint1",
            "panda_joint2",
            "panda_joint3",
            "panda_joint4",
            "panda_joint5",
            "panda_joint6",
            "panda_joint7",
        ];

        fn fk_pose(model: &RobotModel, values: &[f64]) -> Isometry3 {
            let mut state = RobotState::new(model);
            state.set_to_default_values();
            for (name, &v) in PANDA_ARM_JOINTS.iter().zip(values) {
                state.set_variable_position(name, v).unwrap();
            }
            state.update().global_link_transform("panda_link8").unwrap()
        }

        fn satisfaction_rate(
            model: &RobotModel,
            space: &JointModelGroupSpace,
            n: u32,
            seed: u64,
            check: impl Fn(&Posed) -> bool,
        ) -> u32 {
            let mut rng = ChaCha8Rng::seed_from_u64(seed);
            let mut satisfied = 0u32;
            for _ in 0..n {
                let candidate = space.sample_uniform(&mut rng);
                let mut state = RobotState::new(model);
                state.set_to_default_values();
                space.write_robot_state(&candidate, &mut state);
                let posed = state.update();
                if check(&posed) {
                    satisfied += 1;
                }
            }
            satisfied
        }

        let (model, _srdf) = load_panda();
        let space = JointModelGroupSpace::new(&model, "panda_arm").unwrap();
        let tf = Transforms::new("world").unwrap();
        // Scenario 1's own rate is already unmeasurably rare at 20,000
        // draws (see below); scenario 3's needs more draws for the count
        // itself to be a stable estimate rather than noise on a single hit.
        let n1 = 20_000u32;
        let n3 = 200_000u32;

        // Scenario 1's own path constraint, rebuilt identically (same
        // target_pose, same tolerances) to the sweep above.
        let true_values = [0.3, -0.4, 0.2, -1.9, 0.1, 1.2, 0.5];
        let target_pose = fk_pose(&model, &true_values);
        let pc1 = PositionConstraint::new(
            &model,
            &tf,
            "panda_link8",
            "world",
            Vector3::zeros(),
            &[(Shape::Sphere(Sphere::new(0.008).unwrap()), target_pose)],
            1.0,
        )
        .unwrap();
        let oc1 = OrientationConstraint::new(
            &model,
            &tf,
            "panda_link8",
            "world",
            target_pose.rotation,
            OrientationTolerance::RotationVector {
                x: 0.03,
                y: 0.03,
                z: 0.03,
            },
            1.0,
        )
        .unwrap();
        let scenario1_satisfied = satisfaction_rate(&model, &space, n1, 11, |posed| {
            pc1.decide(posed).satisfied && oc1.decide(posed).satisfied
        });

        // Scenario 3's own path constraint, rebuilt identically
        // (orientation-only, position free).
        let oc3 = OrientationConstraint::new(
            &model,
            &tf,
            "panda_link8",
            "world",
            target_pose.rotation,
            OrientationTolerance::RotationVector {
                x: 0.05,
                y: 0.05,
                z: 0.05,
            },
            1.0,
        )
        .unwrap();
        let scenario3_satisfied =
            satisfaction_rate(&model, &space, n3, 12, |posed| oc3.decide(posed).satisfied);

        // Second measurement: not "does an independent global draw land in
        // the corridor" but "starting from a point already known to be
        // inside it, does one step_size-sized random joint perturbation
        // stay inside" -- the quantity RRT-Connect's own tree growth
        // actually depends on (`extend`'s `step_size`-bounded move from an
        // existing tree node), not one-shot global sampling. `true_values`
        // is exactly `target_pose`'s own joint configuration, so it
        // trivially satisfies both scenarios' constraints (0 deviation)
        // and is a valid interior point for both without extra IK solving.
        fn local_step_acceptance_rate(
            model: &RobotModel,
            n: u32,
            seed: u64,
            interior: &[f64],
            step_size: f64,
            check: impl Fn(&Posed) -> bool,
        ) -> u32 {
            let mut rng = ChaCha8Rng::seed_from_u64(seed);
            let mut accepted = 0u32;
            for _ in 0..n {
                let step: Vec<f64> = interior
                    .iter()
                    .map(|&v| v + rng.random_range(-step_size..step_size))
                    .collect();
                let mut state = RobotState::new(model);
                state.set_to_default_values();
                for (name, &v) in PANDA_ARM_JOINTS.iter().zip(&step) {
                    state.set_variable_position(name, v).unwrap();
                }
                let posed = state.update();
                if check(&posed) {
                    accepted += 1;
                }
            }
            accepted
        }

        let step_size = 0.03; // scenario1_budget's own step_size in the sweep above.
        let n_local = 20_000u32;
        let scenario1_local_accepted =
            local_step_acceptance_rate(&model, n_local, 21, &true_values, step_size, |posed| {
                pc1.decide(posed).satisfied && oc1.decide(posed).satisfied
            });
        let scenario3_local_accepted =
            local_step_acceptance_rate(&model, n_local, 21, &true_values, step_size, |posed| {
                oc3.decide(posed).satisfied
            });

        eprintln!(
            "scenario3_orientation_only_corridor_sample_level_satisfaction_rate:\n\
             \x20 global uniform draw:  scenario 1: {scenario1_satisfied}/{n1}, \
             scenario 3: {scenario3_satisfied}/{n3}\n\
             \x20 local step (from true_values, step_size {step_size}): \
             scenario 1: {scenario1_local_accepted}/{n_local}, \
             scenario 3: {scenario3_local_accepted}/{n_local}"
        );

        assert_eq!(
            (
                scenario1_satisfied,
                scenario3_satisfied,
                scenario1_local_accepted,
                scenario3_local_accepted
            ),
            (0, 2, 1837, 16586),
            "moved off the documented sample-level rates"
        );
    }

    /// The D8 equivalence determination this round's brief asked for:
    /// whether a concrete goal state is losslessly representable as "a set
    /// of `JointConstraint`s representing a single state," for the scalar
    /// (non-floating) joint case every fixture group actually exercises.
    ///
    /// One tight (`1e-9`) `JointConstraint` per `panda_arm` variable,
    /// centered on a real target state, covers every group variable
    /// (`joint_coverage_is_full`, `constraint_sampler_manager.rs:321`), so
    /// `select_default_sampler` returns a real `JointConstraintSampler`
    /// (Step A) rather than falling through to the uniform branch —
    /// [`crate::goal_sampler::sample_goal`]'s constrained branch then
    /// samples *every* variable from its own `1e-9`-wide window, converging
    /// on the target to within that same tolerance. This is a direct
    /// measurement, not merely an architectural argument: **for scalar
    /// joints, a concrete state and an N-`JointConstraint` set (one tight
    /// constraint per variable) are interchangeable to the precision the
    /// constraints are given.**
    ///
    /// This does not extend to a floating joint's own local variables
    /// (`trans_x/y/z, rot_x/y/z/w`) without further work: the four rotation
    /// components jointly satisfy a unit-quaternion constraint
    /// (`moveit-model`'s `floating.rs` doc comment) that four independent
    /// per-component `JointConstraint` windows do not preserve at any
    /// nonzero tolerance — only in the tolerance-to-zero limit. This is
    /// reasoned, not measured: no fixture group actually contains a
    /// floating joint's variables to sample and check. Reproduced directly
    /// against every fixture SRDF this crate has (`rg -B2 'type="floating"'
    /// fixtures/*.srdf` finds exactly one, `panda.srdf`'s own
    /// `virtual_joint`; `rg '<joint name="virtual_joint"' fixtures/*.srdf`
    /// finds zero matches — no `<group>` in any fixture SRDF lists it as a
    /// member), so the floating-joint case is an open gap this round leaves
    /// unverified, not one it silently assumed away.
    #[test]
    fn full_joint_constraint_coverage_reconstructs_a_concrete_scalar_state() {
        use moveit_constraints::{Constraint, JointConstraint, select_default_sampler};
        use rand::SeedableRng;
        use rand_chacha::ChaCha8Rng;

        use crate::goal_sampler::sample_goal;

        let (model, srdf) = load_panda();
        let space = JointModelGroupSpace::new(&model, "panda_arm").unwrap();

        let mut target_state = RobotState::new(&model);
        target_state.set_to_default_values();
        let targets = [
            ("panda_joint1", 0.3),
            ("panda_joint2", -0.6),
            ("panda_joint3", 0.9),
            ("panda_joint4", -1.8),
            ("panda_joint5", 0.4),
            ("panda_joint6", 1.9),
            ("panda_joint7", -0.2),
        ];
        for (name, value) in targets {
            target_state.set_variable_position(name, value).unwrap();
        }

        let tolerance = 1e-9;
        let mut goal_constraints = KinematicConstraintSet::new();
        for (name, value) in targets {
            let constraint =
                JointConstraint::new(&model, name, value, tolerance, tolerance, 1.0).unwrap();
            goal_constraints.push(Constraint::Joint(constraint));
        }

        let sampler = select_default_sampler(
            &model,
            "panda_arm",
            goal_constraints.constraints(),
            None,
            vec![],
            4,
        )
        .expect("no subgroup_solvers, so select_default_sampler cannot error here")
        .expect(
            "full joint coverage (one JointConstraint per group variable) must yield a real \
             JointConstraintSampler, not None",
        );

        let mut scene = PlanningScene::new(&model, &srdf);
        let env = ParryCollisionEnv::default();
        let checker = PlanningSceneValidityChecker::new(
            &mut scene,
            &env,
            CollisionRequest::default(),
            None,
            &space,
        );

        let mut rng = ChaCha8Rng::seed_from_u64(0);
        let resolved = sample_goal(
            &space,
            &checker,
            &goal_constraints,
            &target_state,
            Some(sampler.as_ref()),
            &mut rng,
            1,
        )
        .expect("a single attempt must succeed: full coverage guarantees every draw is accepted");

        let mut resolved_state = target_state.clone();
        space.write_robot_state(&resolved, &mut resolved_state);
        for (name, value) in targets {
            let got = resolved_state.variable_position(name).unwrap();
            assert!(
                (got - value).abs() <= tolerance,
                "{name}: resolved {got}, target {value}, outside the {tolerance} tolerance the \
                 constraint set encoded"
            );
        }
    }
}
