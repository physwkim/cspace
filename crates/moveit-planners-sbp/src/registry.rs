// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// No upstream file ported line-for-line: this is a D1/D4-adapted stand-in
// for
//   moveit_core/planning_interface/include/moveit/planning_interface/planning_interface.hpp
// (`PlannerManager`, `PlanningContext`). The registration mechanism
// (`PlannerRegistration` + a `linkme::distributed_slice`) follows
// `moveit_kinematics::registry`'s existing D4 precedent exactly; the
// `PlanningRequest`/`PlanningResponse`/`PlanningContext`/`PlannerManager`
// shapes themselves are new design work, since a motion planner's request
// and a kinematics solver's request have nothing in common upstream.

//! Compile-time [`PlannerManager`] registry and one concrete planner,
//! [`RrtConnectManager`], that plans through a real
//! [`moveit_scene::PlanningScene`] via
//! [`crate::planning_scene_validity::PlanningSceneValidityChecker`].
//!
//! # `PlanningRequest`: the D1-free `MotionPlanRequest` equivalent
//!
//! Upstream's `MotionPlanRequest::goal_constraints` is
//! `Vec<moveit_msgs::msg::Constraints>` — a set of possibly-Cartesian goal
//! constraints, turned into candidate joint-space states at plan time by a
//! `constraint_samplers` sampler
//! (`ConstraintSamplerManager::selectDefaultSampler` picking a
//! `JointConstraintSampler`, an `IKConstraintSampler`, or a
//! `UnionConstraintSampler` of both). Per `PORTING-PLAN.md` D1 (no
//! `moveit_msgs`) this port has no `moveit_msgs::msg::Constraints` to carry
//! in the first place, and — independently of D1 — `constraint_samplers`
//! itself has never been ported (`PORTING-PLAN.md` §3 only nominally scoped
//! it under `moveit-constraints`).
//!
//! **The consequence, settled here rather than left ambiguous: a caller of
//! this crate cannot express a pose (position/orientation) goal at all —
//! this is a missing planning capability, not a missing convenience.**
//! [`PlanningRequest::goal`] is `Vec<CompoundValue>`, and
//! [`crate::rrt_connect::rrt_connect`] takes that as one concrete
//! `S::State`, not a region or a sampler; no code path anywhere in this
//! workspace turns a [`moveit_constraints::PositionConstraint`]/
//! [`moveit_constraints::OrientationConstraint`] into even one candidate
//! joint-space state, let alone the several an IK-backed sampler would offer
//! so the planner could pick whichever is reachable. A caller wanting "move
//! the end-effector to this pose" must invoke `moveit-kinematics` itself,
//! outside this crate, and hand in the single joint-space point one IK call
//! returns — foreclosing the multi-solution goal regions IK is generally
//! many-to-one over, exactly what `IKConstraintSampler` exists to preserve.
//!
//! [`PlanningRequest::goal`] being a concrete state rather than a constraint
//! to sample from was, at the time, the rejected alternative to a
//! single-[`moveit_constraints::JointConstraint`]-per-variable stub: the stub
//! would have silently mishandled any Cartesian goal (exactly the case a
//! real sampler exists to handle), so it would have been indistinguishable
//! from this concrete-state design except for an extra, misleading layer of
//! indirection. That reasoning was correct as far as it went, but stopped one
//! level short of naming what a real sampler would unlock; the paragraph
//! above is that.
//!
//! **Disposition** (round 14: ported, not proposed — the paragraph above
//! describes the state this section used to be written against; it is
//! stale about what has since landed in `moveit-constraints`, corrected
//! here in place rather than deleted, since the still-missing
//! `rrt_connect` half below is still accurate):
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
//! **Round 21** made that second change: [`Goal::Constraints`] carries a
//! [`KinematicConstraintSet`] for the goal (mirroring
//! `ompl_interface::ConstrainedGoalSampler`,
//! `crate::goal_sampler::sample_goal` — see that module's own doc comment
//! for exactly what is and is not ported), resolved to one concrete state
//! before [`crate::rrt_connect::rrt_connect`] starts searching. This closes
//! the joint-constraint case fully: a [`Goal::Constraints`] whose
//! [`moveit_constraints::JointConstraint`]s cover every one of the group's
//! variables gets a real [`moveit_constraints::JointConstraintSampler`]
//! (`select_default_sampler`'s Step A). Through round 22 it did not close
//! the Cartesian-pose case at all: `RrtConnectContext::solve` always passed
//! `solver: None` to `select_default_sampler`, so a
//! [`moveit_constraints::PositionConstraint`]/
//! [`moveit_constraints::OrientationConstraint`]-only [`Goal::Constraints`]
//! built no sampler and fell back to
//! [`crate::space::StateSpace::sample_uniform`] every attempt — not
//! incorrect (`goal_constraints.decide()` still gated acceptance), just
//! practically unable to find a tight Cartesian region by chance within
//! `DEFAULT_MAX_GOAL_SAMPLING_ATTEMPTS` tries.
//!
//! **Round 23** (`PORTING-PLAN.md` §163.3, closing the D12-rejection
//! follow-up §163 left open) added [`PlanningRequest::solver`]: a caller
//! who explicitly constructs a
//! [`moveit_kinematics::KinematicsSolver`] (e.g. from
//! `moveit_kinematics::KINEMATICS_SOLVERS`) and sets this field
//! now gets a real `IKConstraintSampler` for a Cartesian-pose
//! [`Goal::Constraints`], the same way a full joint-constraint goal already
//! did. This is caller-supplied wiring, not automatic resolution — no code
//! anywhere in this crate picks a solver by name or group, matching D4's
//! standing exclusion of that runtime-configuration layer (§68.4/§77.1,
//! reaffirmed by §163's D12 rejection). `None` (every request before this
//! field existed) is unchanged: identical fallback to uniform sampling.
//! Through round 23, [`PlanningRequest::path_constraints`]' own
//! `select_default_sampler` call did **not** read this field: it was
//! consumed via `.take()` inside the goal call only, so the field's meaning
//! ("the caller's solver") silently depended on which call site executed —
//! a path-constraint region needing IK-backed sampling got none, with no
//! type-level signal that this gap existed.
//!
//! **Round 24** (closing the same `PORTING-PLAN.md` §163.3 gap Round 23
//! opened) fixed that: `solve` now converts `self.request.solver` once into
//! a shared `Rc<RefCell<Box<dyn KinematicsSolver>>>>`, and both
//! `select_default_sampler` calls — `path_constraints`' own and the goal's
//! own — go through the same `resolve_constraint_sampler` helper (below,
//! right before this trait's `impl`), wrapped in `SharedKinematicsSolver`,
//! so both are backed by the *same* solver instance instead of only
//! whichever call site got to `.take()` it first. See
//! [`PlanningRequest::solver`]'s own doc comment for why `Rc<RefCell<>>` was
//! chosen over `Arc` and over splitting the field in two.
//!
//! Boundary-tested at the `resolve_constraint_sampler` level
//! (`path_constraints_solver_wiring_matches_the_call_site`, this module's
//! tests) *and*, since round 25's seeding-gap fix
//! (`constrained_sampler::GroupConstraintSampler`'s own doc comment),
//! end-to-end (`path_constraints_end_to_end_wired_vs_unwired`): a wired
//! path sampler now measurably beats an unwired one on a Cartesian-corridor
//! query tight enough to be discriminating.
//!
//! [`PlanningRequest::path_constraints`] *is* carried directly as a
//! [`KinematicConstraintSet`], because path constraints are evaluated
//! per-candidate via `decide()` — see
//! [`crate::planning_scene_validity::PlanningSceneValidityChecker`] — so
//! correctness never depended on a sampler. **Round 20**: `path_constraints`
//! is now also fed to `select_default_sampler` and wired into
//! [`crate::rrt_connect::rrt_connect`]'s uniform-sampling step (not its
//! fixed `goal` — see [`crate::rrt_connect::Sampler`] and
//! [`crate::rrt_connect::ConstrainedStateSampler`], mirroring upstream's
//! `ompl_interface::ConstrainedSampler`), a distinct seam from the goal-region
//! sampling the paragraph above now also describes:
//! `RrtConnectContext::solve` builds the sampler through
//! `crate::constrained_sampler::GroupConstraintSampler` whenever
//! `path_constraints` is `Some`, purely as a sampling-efficiency aid —
//! `checker` below still enforces the constraint on every candidate
//! regardless of whether a sampler was available to help find one.
//!
//! `start` is not a [`PlanningRequest`] field: [`RrtConnectManager::get_planning_context`]
//! reads it from the [`moveit_scene::PlanningScene`] it is given
//! (`scene.current_state()`), matching how upstream planning normally seeds
//! from the scene's current state rather than duplicating it into the
//! request.
//!
//! Planner-specific tuning ([`crate::rrt_connect::RrtConnectParams`],
//! [`PlanningRequest::resolution`]) is a concretely-typed field rather than
//! upstream's stringly-typed `PlannerConfigurationSettings::config:
//! HashMap<String, String>` bag: this port's compile-time registry already
//! knows which concrete planner it is constructing a request for, so there
//! is no runtime plugin boundary for a string bag to cross.

use std::cell::RefCell;
use std::rc::Rc;

use moveit_collision::{CollisionRequest, ParryCollisionEnv};
use moveit_constraints::{
    Constraint, ConstraintSampler, KinematicConstraintSet, select_default_sampler,
};
use moveit_geometry::Isometry3;
use moveit_kinematics::{KinematicsSolver, SolveOptions};
use moveit_model::RobotModel;
use moveit_scene::PlanningScene;
use moveit_state::RobotState;
use rand::SeedableRng;
use rand_chacha::ChaCha8Rng;

use crate::compound::CompoundValue;
use crate::constrained_sampler::GroupConstraintSampler;
use crate::error::SbpError;
use crate::joint_model_group_space::JointModelGroupSpace;
use crate::planning_scene_validity::PlanningSceneValidityChecker;
use crate::rrt_connect::{
    ConstrainedStateSampler, PlanningFailure, RrtConnectParams, Sampler, rrt_connect,
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

/// A [`PlanningRequest`]'s goal.
///
/// Upstream never has this split at the `MotionPlanRequest` level — a goal
/// is always `goal_constraints: Vec<moveit_msgs::msg::Constraints>`, and
/// whether that ends up sampled (`ompl_interface::ConstrainedGoalSampler`)
/// or driving the fixed-state case directly is decided deeper in the OMPL
/// planning context, invisibly to the caller. This port's
/// [`crate::rrt_connect::rrt_connect`] instead takes one concrete
/// `S::State` as `goal`, so *something* has to resolve a constraint region
/// down to that one state before the search starts — [`Goal`] is that
/// choice, made explicit at the [`PlanningRequest`] boundary instead of
/// buried in `solve()`.
#[derive(Debug, Clone)]
pub enum Goal {
    /// A single concrete target state, in the group's own
    /// [`crate::joint_model_group_space::JointModelGroupSpace`] shape —
    /// this crate's only goal shape before round 21. See this module's doc
    /// comment's "consequence" paragraph for what a caller still cannot
    /// express even with [`Goal::Constraints`] available alongside it
    /// (namely: this variant remains the only way to reach a state a
    /// sampler cannot find on its own, e.g. because no
    /// `moveit_kinematics::KinematicsSolver` was supplied for an
    /// IK-backed pose goal).
    State(Vec<CompoundValue>),
    /// A goal region expressed as constraints, resolved to one concrete
    /// state by `goal_sampler::sample_goal` before
    /// [`crate::rrt_connect::rrt_connect`] runs. See that function's own
    /// module doc comment for exactly which parts of
    /// `ompl_interface::ConstrainedGoalSampler` this reproduces, and which
    /// it deliberately does not (in particular: this resolves to *one*
    /// state, never a lazily-grown region of up to ten).
    Constraints(KinematicConstraintSet),
}

/// A motion planning query. See this module's doc comment for why this,
/// rather than a transcription of upstream's `MotionPlanRequest`, is the
/// shape here.
pub struct PlanningRequest {
    /// The [`moveit_model::JointModelGroup`] to plan for — looked up
    /// against `scene.robot_model()` by
    /// [`RrtConnectManager::get_planning_context`].
    pub group_name: String,
    /// The target. See [`Goal`]'s own doc comment for the two shapes this
    /// can take and why.
    pub goal: Goal,
    /// Constraints every waypoint (not just the goal) must satisfy, mirroring
    /// upstream's `path_constraints`. `None` means unconstrained.
    pub path_constraints: Option<KinematicConstraintSet>,
    /// [`crate::validity::DiscreteMotionValidator`]'s bisection resolution,
    /// in the group's own [`crate::space::StateSpace::distance`] units.
    pub resolution: f64,
    /// Seeds this query's RNG — see [`crate::rrt_connect::rrt_connect`]'s
    /// determinism guarantee under [`crate::rrt_connect::Termination::Iterations`].
    pub seed: u64,
    /// RRT-Connect's own tuning parameters.
    pub params: RrtConnectParams,
    /// Backs *every* [`select_default_sampler`] call `RrtConnectContext::solve`
    /// makes — both [`Goal::Constraints`]' own and
    /// [`PlanningRequest::path_constraints`]' own — with one real IK solver,
    /// so a [`moveit_constraints::PositionConstraint`]/
    /// [`moveit_constraints::OrientationConstraint`] region gets a real
    /// `IKConstraintSampler` instead of always falling back to uniform
    /// sampling. See this module's own doc comment ("Round 21"/"Round 24"
    /// paragraphs) for the gap this closes, and `PORTING-PLAN.md`
    /// §163.3/§164.5 for why this is caller-supplied wiring, not
    /// automatic resolution: **nothing in this crate picks a solver by
    /// name.** A caller wanting one must construct it themselves, e.g. from
    /// `moveit_kinematics::KINEMATICS_SOLVERS`, exactly as D4 already
    /// requires everywhere else in this workspace. `None` (every call
    /// site's behavior before this field existed) remains fully valid and
    /// keeps producing identical results.
    ///
    /// **What is verified for the `path_constraints` call site
    /// specifically:** `registry.rs`'s
    /// `path_constraints_solver_wiring_matches_the_call_site` test proves
    /// `resolve_constraint_sampler` (the function both call sites now share)
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
    /// already-verified behavior for [`Goal::Constraints`]. Setting this
    /// field is always safe either way — `checker` still enforces
    /// `path_constraints` on every candidate regardless of whether a
    /// sampler helped find one.
    ///
    /// # Why `Option<Box<dyn KinematicsSolver>>`, not `Rc`/`Arc`/two fields
    ///
    /// The caller-facing type stays a plain owned `Box`: a caller building
    /// one [`PlanningRequest`] never needs to know this context internally
    /// shares it across two `select_default_sampler` calls.
    /// `RrtConnectContext::solve` itself does the one-time conversion into
    /// `SharedKinematicsSolver` (backed by `Rc<RefCell<Box<dyn
    /// KinematicsSolver>>>` — the exact type `select_default_sampler`
    /// already builds internally per call,
    /// `moveit-constraints/src/constraint_sampler_manager.rs`'s own Step B),
    /// so this field always means the same thing regardless of which
    /// call site reads it: "the caller's solver, if any." Before round 24
    /// this field read `Box<dyn KinematicsSolver>` and was consumed via
    /// `.take()` inside the goal-only call site — meaning depended on
    /// which branch executed, and a second call site added later would
    /// have silently observed `None`; that is the exact defect round 24
    /// closes.
    ///
    /// Three ownership shapes were weighed for the *internal* sharing
    /// mechanism `solve` converts into:
    ///
    /// - **`Rc<RefCell<Box<dyn KinematicsSolver>>>>` (chosen).** Every
    ///   [`PlanningContext`] in this crate is already documented
    ///   single-threaded (see that trait's own doc comment), so this needs
    ///   no `Send`/`Sync` bound on [`KinematicsSolver`] — none is declared on
    ///   the trait today, so requiring one would newly constrain every
    ///   implementation in this workspace. It also matches the sharing
    ///   mechanism `select_default_sampler` already builds internally per
    ///   call (`constraint_sampler_manager.rs`'s own Step B), so this round
    ///   introduces no new pattern, just reuses it across call sites.
    /// - **`Arc<dyn KinematicsSolver>`.** Rejected on two grounds, not just
    ///   the `Send + Sync` cost: [`KinematicsSolver::solve_with_options`]
    ///   takes `&mut self`, and a bare `Arc` grants only shared (immutable)
    ///   access — reaching `&mut self` through it needs interior mutability
    ///   regardless (`Arc<Mutex<Box<dyn KinematicsSolver>>>>`, not a plain
    ///   `Arc`), at which point it is the identical shape as the chosen one
    ///   plus an unneeded `Send + Sync` bound and mutex overhead for a
    ///   single-threaded caller.
    /// - **Splitting into two fields** (one solver per call site). Rejected:
    ///   it removes the dual meaning but forces a caller with one real
    ///   solver to either construct it twice or clone a handle themselves —
    ///   pushing the exact sharing problem this type solves onto every
    ///   caller instead of solving it once, centrally.
    pub solver: Option<Box<dyn KinematicsSolver>>,
}

/// Forwards to a solver borrowed through `Rc<RefCell<Box<dyn
/// KinematicsSolver>>>` so more than one owner can hold a
/// [`KinematicsSolver`]-shaped handle onto the *same* underlying solver
/// instance — see [`PlanningRequest::solver`]'s own doc comment for why
/// [`RrtConnectContext::solve`] needs this.
///
/// The four `&str`/`&[String]`-returning accessors
/// ([`KinematicsSolver::group_name`], [`KinematicsSolver::joint_names`],
/// [`KinematicsSolver::base_frame`], [`KinematicsSolver::tip_frame`]) are
/// cached as owned `String`/`Vec<String>` at construction time rather than
/// forwarded live through `RefCell::borrow()`: a borrow guard is a temporary
/// that cannot outlive the method call producing it, so `&str` borrowed
/// through one cannot satisfy `fn group_name(&self) -> &str`'s `&self`
/// lifetime (`E0515`). Only [`KinematicsSolver::solve_with_options`] forwards
/// live, through `RefCell::borrow_mut()`, because it returns an owned
/// `Option<Vec<f64>>` with no such lifetime problem.
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

impl std::fmt::Debug for PlanningRequest {
    /// Manual, not derived: [`moveit_kinematics::KinematicsSolver`] has no
    /// `Debug` bound (nothing here needs one), so `solver` cannot go
    /// through `#[derive(Debug)]` — printed as presence only.
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("PlanningRequest")
            .field("group_name", &self.group_name)
            .field("goal", &self.goal)
            .field("path_constraints", &self.path_constraints)
            .field("resolution", &self.resolution)
            .field("seed", &self.seed)
            .field("params", &self.params)
            .field(
                "solver",
                &self.solver.as_ref().map(|_| "Box<dyn KinematicsSolver>"),
            )
            .finish()
    }
}

/// A successful plan: one [`RobotState`] per waypoint, in order, first
/// equal to the request's start (`scene.current_state()`) and last equal to
/// [`PlanningRequest::goal`] — the concrete state itself for
/// [`Goal::State`], or whatever `crate::goal_sampler::sample_goal`
/// resolved for [`Goal::Constraints`].
#[derive(Debug, Clone)]
pub struct PlanningResponse<'m> {
    /// The waypoints, in traversal order.
    pub trajectory: Vec<RobotState<'m>>,
}

/// Everything that can go wrong building or running a [`PlanningContext`].
#[derive(Debug, thiserror::Error)]
#[non_exhaustive]
pub enum PlanError {
    /// [`RrtConnectManager::get_planning_context`] was given a
    /// [`PlanningRequest::group_name`] the scene's `RobotModel` does not
    /// have, or another boundary-input error from this crate.
    #[error(transparent)]
    Sbp(#[from] SbpError),
    /// [`PlanningContext::solve`] ran but did not find a path.
    #[error("planning failed: {0}")]
    Failed(#[from] PlanningFailure),
    /// [`Goal::Constraints`] could not be resolved to a single concrete
    /// state within `DEFAULT_MAX_GOAL_SAMPLING_ATTEMPTS` attempts.
    /// Mirrors `ConstrainedGoalSampler::sampleUsingConstraintSampler`
    /// returning `false` after `attempts_so_far >= max_attempts`
    /// (`constrained_goal_sampler.cpp:102-103`) — upstream surfaces that
    /// through `GoalLazySamples`'s own empty-goal-region timeout deep in
    /// OMPL; this port reports it directly instead.
    #[error("no goal state satisfying the goal constraints was found within the sampling budget")]
    NoGoalSample,
}

/// Replaces upstream `planning_interface::PlanningContext`: a planning
/// query bound to a scene, ready to run.
///
/// # Deviation from upstream: no `terminate`/`clear`
///
/// Upstream's `PlanningContext` supports being handed to one thread while
/// `terminate()` is called from another (asynchronous cancellation) and
/// being reused for a second, unrelated request via `clear()`. Every
/// [`PlanningContext`] here is single-use and single-threaded — the same
/// scope discipline [`crate::rrt_connect::rrt_connect`] itself already
/// has (see [`crate::planning_scene_validity::PlanningSceneValidityChecker`]'s
/// `# Why RefCell` section) — so neither method has a caller to serve yet;
/// adding them now would be speculative API surface for a concurrency model
/// this crate does not have.
pub trait PlanningContext<'m> {
    /// Runs the query to completion (this crate's planners are synchronous;
    /// see this trait's `# Deviation` for why there is no separate
    /// `terminate`).
    fn solve(&mut self) -> Result<PlanningResponse<'m>, PlanError>;
}

/// Replaces upstream `planning_interface::PlannerManager`: builds a
/// [`PlanningContext`] for a `(scene, request)` pair.
///
/// # Deviation from upstream: specialized to [`ParryCollisionEnv`]
///
/// Upstream's `PlannerManager` is not itself generic over the collision
/// checker — the scene it is given already owns one. This port's
/// [`moveit_scene::PlanningScene`] is generic over `E: CollisionEnv<..>`
/// instead of owning one (see that type's own doc comment), which would
/// force `get_planning_context` to be generic over `E` too — and a generic
/// *type* parameter on a trait method breaks `dyn` object-safety (a generic
/// *lifetime* parameter, like this method's `'a`/`'m`, does not).
/// [`moveit_collision::ParryCollisionEnv`] is the only [`moveit_collision::CollisionEnv`]
/// implementation anywhere in this workspace (`PORTING-PLAN.md` D4.5:
/// parry3d-f64 replaces FCL+Bullet outright, not as one plugin among
/// several), so specializing directly to it costs nothing today and keeps
/// [`PLANNER_MANAGERS`] usable as `[dyn PlannerManager]` rather than
/// requiring a type parameter on the registry itself.
pub trait PlannerManager {
    /// This manager's name, matching [`PlannerRegistration::name`].
    fn name(&self) -> &'static str;

    /// Builds a [`PlanningContext`] that will plan `request` against
    /// `scene` using `env` for collision. Fails only if `request` cannot be
    /// resolved against `scene.robot_model()` (currently: an unknown
    /// [`PlanningRequest::group_name`]) — planning failure itself surfaces
    /// from [`PlanningContext::solve`], not here, matching upstream's own
    /// split between context construction and `solve()`.
    fn get_planning_context<'a, 'm>(
        &self,
        scene: &'a mut PlanningScene<'m>,
        env: &'a ParryCollisionEnv,
        request: PlanningRequest,
    ) -> Result<Box<dyn PlanningContext<'m> + 'a>, PlanError>;
}

/// One [`PlannerManager`] implementation's compile-time registration.
/// Replaces upstream's `CLASS_LOADER_REGISTER_CLASS(ConcreteType,
/// planning_interface::PlannerManager)` — see
/// `moveit_kinematics::registry::SolverRegistration`'s doc comment, which
/// this mirrors exactly for the same D4 reason.
pub struct PlannerRegistration {
    /// The name a caller scanning [`PLANNER_MANAGERS`] matches on.
    /// `"rrt_connect"` for this crate's own planner.
    pub name: &'static str,
    /// Builds one instance of this registration's [`PlannerManager`].
    pub construct: fn() -> Box<dyn PlannerManager>,
}

/// Every [`PlannerManager`] this crate (or, later, another crate linked
/// into the same binary) registers. See [`PlannerRegistration`]'s doc
/// comment, and `Cargo.toml`'s `[lints.rust]` comment for why this crate
/// sets `unsafe_code = "allow"` to host it.
#[linkme::distributed_slice]
pub static PLANNER_MANAGERS: [PlannerRegistration];

/// [`PlannerManager`] for [`crate::rrt_connect::rrt_connect`].
#[derive(Debug, Default)]
pub struct RrtConnectManager;

impl PlannerManager for RrtConnectManager {
    fn name(&self) -> &'static str {
        "rrt_connect"
    }

    fn get_planning_context<'a, 'm>(
        &self,
        scene: &'a mut PlanningScene<'m>,
        env: &'a ParryCollisionEnv,
        request: PlanningRequest,
    ) -> Result<Box<dyn PlanningContext<'m> + 'a>, PlanError> {
        let space = JointModelGroupSpace::new(scene.robot_model(), &request.group_name)?;
        Ok(Box::new(RrtConnectContext {
            scene,
            env,
            space,
            request,
        }))
    }
}

struct RrtConnectContext<'a, 'm> {
    scene: &'a mut PlanningScene<'m>,
    env: &'a ParryCollisionEnv,
    space: JointModelGroupSpace,
    request: PlanningRequest,
}

/// The `select_default_sampler` call both the `path_constraints` and the
/// goal (`Goal::Constraints`) branches of [`RrtConnectContext::solve`] make
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

impl<'a, 'm> PlanningContext<'m> for RrtConnectContext<'a, 'm> {
    fn solve(&mut self) -> Result<PlanningResponse<'m>, PlanError> {
        let start = self.space.read_robot_state(self.scene.current_state());
        let template = self.scene.current_state().clone();

        // Round 24: `self.request.solver` is converted to a shared
        // `Rc<RefCell<Box<dyn KinematicsSolver>>>>` once, up front, so both
        // `resolve_constraint_sampler` calls below (`path_constraints`' own
        // and the goal's own) can be backed by the *same* real IK solver
        // instance instead of only whichever call site got to `.take()` it
        // first. See `PlanningRequest::solver`'s own doc comment and
        // `SharedKinematicsSolver` for why this replaces the old
        // single-consumer `.take()` pattern.
        let shared_solver: Option<Rc<RefCell<Box<dyn KinematicsSolver>>>> = self
            .request
            .solver
            .take()
            .map(|solver| Rc::new(RefCell::new(solver)));

        let constraint_sampler = self
            .request
            .path_constraints
            .as_ref()
            .and_then(|constraints| {
                resolve_constraint_sampler(
                    self.scene.robot_model(),
                    &self.request.group_name,
                    constraints.constraints(),
                    shared_solver.clone(),
                )
            });
        let path_sampler = constraint_sampler
            .as_deref()
            .map(|sampler| GroupConstraintSampler::new(&self.space, sampler, template.clone()));

        let goal_constraint_sampler = match &self.request.goal {
            Goal::Constraints(goal_constraints) => resolve_constraint_sampler(
                self.scene.robot_model(),
                &self.request.group_name,
                goal_constraints.constraints(),
                shared_solver.clone(),
            ),
            Goal::State(_) => None,
        };

        let checker = PlanningSceneValidityChecker::new(
            &mut *self.scene,
            self.env,
            CollisionRequest::default(),
            self.request.path_constraints.as_ref(),
            &self.space,
        );
        let motion_validator = DiscreteMotionValidator::new(&checker, self.request.resolution);
        let mut rng = ChaCha8Rng::seed_from_u64(self.request.seed);

        let goal = match &self.request.goal {
            Goal::State(state) => state.clone(),
            Goal::Constraints(goal_constraints) => crate::goal_sampler::sample_goal(
                &self.space,
                &checker,
                goal_constraints,
                &template,
                goal_constraint_sampler.as_deref(),
                &mut rng,
                DEFAULT_MAX_GOAL_SAMPLING_ATTEMPTS,
            )
            .ok_or(PlanError::NoGoalSample)?,
        };

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
            &self.request.params,
        )?;

        let trajectory = path
            .into_iter()
            .map(|state| {
                let mut robot_state = template.clone();
                self.space.write_robot_state(&state, &mut robot_state);
                robot_state
            })
            .collect();

        Ok(PlanningResponse { trajectory })
    }
}

#[linkme::distributed_slice(PLANNER_MANAGERS)]
static RRT_CONNECT: PlannerRegistration = PlannerRegistration {
    name: "rrt_connect",
    construct: || Box::new(RrtConnectManager),
};

#[cfg(test)]
mod tests {
    use std::fs;
    use std::sync::Arc;

    use moveit_collision::LinkPaddingScale;
    use moveit_geometry::{Cuboid, Isometry3, Shape};
    use moveit_model::{MeshSearchPaths, RobotModel};
    use moveit_srdf::SrdfModel;
    use rand::RngExt;

    use super::*;
    use crate::rrt_connect::Termination;
    use crate::space::StateSpace;

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

    fn default_params(seed: u64) -> (f64, u64, RrtConnectParams) {
        (
            0.05,
            seed,
            RrtConnectParams {
                step_size: 0.5,
                goal_bias: 0.05,
                termination: Termination::Iterations(20_000),
                nn_degree: 8,
            },
        )
    }

    #[test]
    fn rrt_connect_is_findable_by_name_in_the_registry() {
        let registration = PLANNER_MANAGERS
            .iter()
            .find(|r| r.name == "rrt_connect")
            .expect("RrtConnectManager must be registered under \"rrt_connect\"");
        let manager = (registration.construct)();
        assert_eq!(manager.name(), "rrt_connect");
    }

    #[test]
    fn unknown_group_is_rejected_before_any_search_runs() {
        let (model, srdf) = load_panda();
        let mut scene = PlanningScene::new(&model, &srdf);
        let env = ParryCollisionEnv::default();
        let manager = RrtConnectManager;
        let (resolution, seed, params) = default_params(0);

        let request = PlanningRequest {
            group_name: "not_a_real_group".to_string(),
            goal: Goal::State(vec![]),
            path_constraints: None,
            resolution,
            seed,
            params,
            solver: None,
        };

        let result = manager.get_planning_context(&mut scene, &env, request);
        assert!(matches!(
            result,
            Err(PlanError::Sbp(SbpError::UnknownGroup { .. }))
        ));
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
        let manager = RrtConnectManager;
        let (resolution, seed, params) = default_params(1);

        let space = JointModelGroupSpace::new(&model, "panda_arm").unwrap();
        let mut rng = ChaCha8Rng::seed_from_u64(3);
        let goal = space.sample_uniform(&mut rng);
        // Captured before `solve()`: `PlanningSceneValidityChecker::is_valid`
        // leaves the scene's current state at whatever it last checked (see
        // its own doc comment's `# Side effect`), so reading it back
        // afterward would not recover the actual start the query ran from.
        let expected_start = space.read_robot_state(scene.current_state());

        let request = PlanningRequest {
            group_name: "panda_arm".to_string(),
            goal: Goal::State(goal.clone()),
            path_constraints: None,
            resolution,
            seed,
            params,
            solver: None,
        };

        let mut context = manager
            .get_planning_context(&mut scene, &env, request)
            .expect("panda_arm is a real group");
        let response = context
            .solve()
            .expect("an empty-world panda_arm query must be solvable");
        drop(context);

        assert!(response.trajectory.len() >= 2);
        let start_positions = space.read_robot_state(&response.trajectory[0]);
        let end_positions = space.read_robot_state(response.trajectory.last().unwrap());
        assert_eq!(
            end_positions, goal,
            "the last waypoint must equal the requested goal exactly"
        );
        assert_eq!(
            start_positions, expected_start,
            "the first waypoint must equal the scene's start state"
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

        let manager = RrtConnectManager;
        let request = PlanningRequest {
            group_name: "body_group".to_string(),
            goal: Goal::State(goal.clone()),
            path_constraints: None,
            resolution: 0.02,
            seed: 3,
            params: RrtConnectParams {
                step_size: 0.3,
                goal_bias: 0.1,
                termination: Termination::Iterations(50_000),
                nn_degree: 8,
            },
            solver: None,
        };

        let mut context = manager
            .get_planning_context(&mut scene, &env, request)
            .expect("body_group is a real group");
        let response = context
            .solve()
            .expect("a wall with clear space on both sides must be solvable");
        drop(context);

        assert!(
            response.trajectory.len() > 2,
            "a direct 2-waypoint path would cross the wall; a valid solution must detour"
        );

        let validity = scene.is_path_valid(
            &env,
            &CollisionRequest::default(),
            &response.trajectory,
            None,
            &[],
        );
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
            let manager = RrtConnectManager;
            let request = PlanningRequest {
                group_name: "panda_arm".to_string(),
                goal: Goal::State(goal.clone()),
                path_constraints: Some(path_constraints),
                resolution: 0.05,
                seed,
                params: small_budget.clone(),
                solver: None,
            };
            let mut context = manager
                .get_planning_context(&mut scene, &env, request)
                .expect("panda_arm is a real group");
            let response = context.solve().unwrap_or_else(|e| {
                panic!(
                    "seed {seed}: the wired search must solve within the same iteration budget \
                     the unwired control above exhausts; got {e:?}"
                )
            });
            drop(context);

            for (index, waypoint) in response.trajectory.iter().enumerate() {
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
        let manager = RrtConnectManager;
        let (resolution, seed, params) = default_params(2);

        let joint_constraint = JointConstraint::new(&model, "panda_joint1", 0.0, 0.001, 0.001, 1.0)
            .expect("valid joint constraint");
        let mut goal_constraints = KinematicConstraintSet::new();
        goal_constraints.push(Constraint::Joint(joint_constraint));

        let request = PlanningRequest {
            group_name: "panda_arm".to_string(),
            goal: Goal::Constraints(goal_constraints),
            path_constraints: None,
            resolution,
            seed,
            params,
            solver: None,
        };

        let mut context = manager
            .get_planning_context(&mut scene, &env, request)
            .expect("panda_arm is a real group");
        let response = context
            .solve()
            .expect("an empty-world panda_arm query with a satisfiable goal region must solve");
        drop(context);

        let value = response
            .trajectory
            .last()
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
            let manager = RrtConnectManager;
            let (resolution, req_seed, params) = default_params(seed);
            let unwired_request = PlanningRequest {
                group_name: "panda_arm".to_string(),
                goal: Goal::Constraints(build_goal_constraints()),
                path_constraints: None,
                resolution,
                seed: req_seed,
                params: params.clone(),
                solver: None,
            };
            let mut unwired_context = manager
                .get_planning_context(&mut unwired_scene, &env, unwired_request)
                .expect("panda_arm is a real group");
            let unwired_result = unwired_context.solve();
            assert!(
                matches!(unwired_result, Err(PlanError::NoGoalSample)),
                "seed {seed}: solver: None must still fail to resolve a Cartesian-only goal \
                 (no full joint coverage, no solver -- select_default_sampler builds nothing, \
                 and 1000 uniform 7-DOF samples essentially never land inside a 0.02/0.1 \
                 Cartesian window), matching this call site's behaviour before \
                 PlanningRequest::solver existed; got {unwired_result:?}"
            );
            drop(unwired_context);

            let mut wired_scene = PlanningScene::new(&model, &srdf);
            let solver: Box<dyn KinematicsSolver> = Box::new(
                NewtonRaphsonSolver::new(&model, "panda_arm", &SolverParams::default())
                    .expect("panda_arm is a chain"),
            );
            let wired_request = PlanningRequest {
                group_name: "panda_arm".to_string(),
                goal: Goal::Constraints(build_goal_constraints()),
                path_constraints: None,
                resolution,
                seed: req_seed,
                params,
                solver: Some(solver),
            };
            let mut wired_context = manager
                .get_planning_context(&mut wired_scene, &env, wired_request)
                .expect("panda_arm is a real group");
            let response = wired_context
                .solve()
                .unwrap_or_else(|e| panic!("seed {seed}: solver: Some(..) must resolve the same goal the unwired control above could not; got {e:?}"));
            drop(wired_context);

            let mut last = response.trajectory.last().unwrap().clone();
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
    /// **Measured after the fix: unwired 1/5, wired 5/5** (`step_size:
    /// 0.03`, `goal_bias: 0.0`, `Iterations(20)`) -- the reverse of the
    /// pre-fix regression this scenario's ancestor recorded. This is one
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
            let manager = RrtConnectManager;
            let unwired_request = PlanningRequest {
                group_name: "panda_arm".to_string(),
                goal: Goal::State(goal.clone()),
                path_constraints: Some(build_path_constraints()),
                resolution: 0.05,
                seed,
                params: small_budget.clone(),
                solver: None,
            };
            let mut unwired_context = manager
                .get_planning_context(&mut unwired_scene, &env, unwired_request)
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
            let wired_request = PlanningRequest {
                group_name: "panda_arm".to_string(),
                goal: Goal::State(goal.clone()),
                path_constraints: Some(build_path_constraints()),
                resolution: 0.05,
                seed,
                params: small_budget.clone(),
                solver: Some(wired_solver),
            };
            let mut wired_context = manager
                .get_planning_context(&mut wired_scene, &env, wired_request)
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
            (1, 5),
            "moved off the documented unwired 1/5, wired 5/5"
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
    /// scenario 1 (self-motion, Goal::State):          unwired 1/5, wired 5/5
    /// scenario 2 (Goal::Constraints, no corridor):     unwired 0/5, wired 5/5
    /// scenario 3 (orientation-only corridor):          unwired 5/5, wired 5/5
    /// scenario 4, tight  (0.03/Iterations(20)):        unwired 1/5, wired 5/5
    /// scenario 4, medium (0.1 /Iterations(20)):        unwired 0/5, wired 5/5
    /// scenario 4, loose  (0.2 /Iterations(200)):       unwired 0/5, wired 5/5
    /// ```
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
        /// `goal`/`path_constraints` are factories, not values, because
        /// `PlanningRequest` cannot derive `Clone` (`solver` is `Option<Box<dyn
        /// KinematicsSolver>>`) -- each of the `2 * num_seeds` requests built
        /// needs its own independently owned copy.
        fn run_scenario(
            model: &RobotModel,
            srdf: &SrdfModel,
            start_values: &[f64],
            goal: impl Fn() -> Goal,
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
            let manager = RrtConnectManager;
            let mut wired_successes = 0u32;
            let mut unwired_successes = 0u32;
            for seed in 0..num_seeds {
                let mut unwired_scene = PlanningScene::new(model, srdf);
                set_start(&mut unwired_scene);
                let unwired_request = PlanningRequest {
                    group_name: "panda_arm".to_string(),
                    goal: goal(),
                    path_constraints: path_constraints(),
                    resolution: 0.05,
                    seed,
                    params: budget.clone(),
                    solver: None,
                };
                let mut unwired_context = manager
                    .get_planning_context(&mut unwired_scene, &env, unwired_request)
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
                let wired_request = PlanningRequest {
                    group_name: "panda_arm".to_string(),
                    goal: goal(),
                    path_constraints: path_constraints(),
                    resolution: 0.05,
                    seed,
                    params: budget.clone(),
                    solver: Some(wired_solver),
                };
                let mut wired_context = manager
                    .get_planning_context(&mut wired_scene, &env, wired_request)
                    .expect("panda_arm is a real group");
                if wired_context.solve().is_ok() {
                    wired_successes += 1;
                }
                drop(wired_context);
            }
            (wired_successes, unwired_successes)
        }

        let (model, srdf) = load_panda();
        let space = JointModelGroupSpace::new(&model, "panda_arm").unwrap();
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
            Goal::State(space.read_robot_state(&goal_state))
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
            Goal::Constraints(set)
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
            Goal::State(space.read_robot_state(&goal_state))
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
            (1, 5),
            "scenario 1 (self-motion, Goal::State) moved off the documented unwired 1/5, wired 5/5"
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
            ("tight (0.03/Iterations(20))", 5u32, 1u32),
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
