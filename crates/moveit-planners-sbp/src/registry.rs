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
//! capability gap above, and still has not: [`crate::rrt_connect::rrt_connect`]'s
//! `goal` parameter is one fixed `S::State`, not a region or a
//! re-sampleable source, so even though `IkConstraintSamplerAdapter` now
//! exists, it has nowhere in this crate to hand its (potentially many,
//! potentially retried-on-collision) candidate states — RRT-Connect's
//! *goal* still needs a second change, accepting something
//! `GoalSampleableRegion`-shaped, before it could actually consume one.
//! That second change has not been made; this disposition note describes
//! the sampler side only.
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
//! `ompl_interface::ConstrainedSampler`), a distinct seam from the
//! still-unaddressed goal-region gap the paragraph above describes:
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

use moveit_collision::{CollisionRequest, ParryCollisionEnv};
use moveit_constraints::{
    DEFAULT_MAX_SAMPLING_ATTEMPTS, KinematicConstraintSet, select_default_sampler,
};
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

/// A motion planning query. See this module's doc comment for why this,
/// rather than a transcription of upstream's `MotionPlanRequest`, is the
/// shape here.
#[derive(Debug, Clone)]
pub struct PlanningRequest {
    /// The [`moveit_model::JointModelGroup`] to plan for — looked up
    /// against `scene.robot_model()` by
    /// [`RrtConnectManager::get_planning_context`].
    pub group_name: String,
    /// The target state, in the group's own
    /// [`crate::joint_model_group_space::JointModelGroupSpace`] shape. See
    /// this module's doc comment for why this is a concrete state rather
    /// than a constraint to sample from.
    pub goal: Vec<CompoundValue>,
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
}

/// A successful plan: one [`RobotState`] per waypoint, in order, first
/// equal to the request's start (`scene.current_state()`) and last equal to
/// [`PlanningRequest::goal`].
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

impl<'a, 'm> PlanningContext<'m> for RrtConnectContext<'a, 'm> {
    fn solve(&mut self) -> Result<PlanningResponse<'m>, PlanError> {
        let start = self.space.read_robot_state(self.scene.current_state());
        let template = self.scene.current_state().clone();

        // Mirrors `ompl_interface::ModelBasedPlanningContext::allocPathConstrainedSampler`
        // (`model_based_planning_context.cpp`): builds the sampler
        // `ConstraintSamplerManager::selectDefaultSampler` picks for
        // `path_constraints`, before the tree search starts, so every
        // uniform sample it draws can be constraint-directed rather than
        // rejection-sampled after the fact. `select_default_sampler`'s only
        // `Err` is an unresolvable name inside `subgroup_solvers`
        // (`constraint_sampler_manager.rs:262`) — structurally unreachable
        // here since `subgroup_solvers` is always empty: this port has no
        // per-request IK-solver-per-subgroup wiring (see this module's own
        // doc comment on why `PlanningRequest` carries no such thing).
        // `Ok(None)` (no sampler could be built — e.g. `path_constraints`
        // has no joint constraint and `solver: None` forecloses the
        // IK-backed position/orientation path) is not an error: it means
        // this query falls back to plain uniform sampling, exactly as if
        // `path_constraints` were absent from the sampler's point of view.
        // Correctness does not depend on a sampler existing either way —
        // `checker` below still enforces `path_constraints` on every
        // candidate regardless.
        let constraint_sampler = self
            .request
            .path_constraints
            .as_ref()
            .and_then(|constraints| {
                select_default_sampler(
                    self.scene.robot_model(),
                    &self.request.group_name,
                    constraints.constraints(),
                    None,
                    vec![],
                    DEFAULT_MAX_SAMPLING_ATTEMPTS,
                )
                .expect(
                    "select_default_sampler's only Err is an unresolvable subgroup_solvers name; \
                 subgroup_solvers is always empty here, so that path can never be taken",
                )
            });
        let path_sampler = constraint_sampler
            .as_deref()
            .map(|sampler| GroupConstraintSampler::new(&self.space, sampler, template.clone()));

        let checker = PlanningSceneValidityChecker::new(
            &mut *self.scene,
            self.env,
            CollisionRequest::default(),
            self.request.path_constraints.as_ref(),
            &self.space,
        );
        let motion_validator = DiscreteMotionValidator::new(&checker, self.request.resolution);
        let mut rng = ChaCha8Rng::seed_from_u64(self.request.seed);

        let path = rrt_connect(
            &self.space,
            &checker,
            &motion_validator,
            start,
            self.request.goal.clone(),
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
            goal: vec![],
            path_constraints: None,
            resolution,
            seed,
            params,
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
            goal: goal.clone(),
            path_constraints: None,
            resolution,
            seed,
            params,
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
            goal: goal.clone(),
            path_constraints: None,
            resolution: 0.02,
            seed: 3,
            params: RrtConnectParams {
                step_size: 0.3,
                goal_bias: 0.1,
                termination: Termination::Iterations(50_000),
                nn_degree: 8,
            },
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
    /// swept combination actually used below (`+/-0.005`, 20 iterations)
    /// scored 30/30 unwired failures *and* 30/30 wired successes across
    /// seeds `0..30` — see this round's git history for the sweep.
    ///
    /// - **Unwired control**: [`rrt_connect`] called directly with
    ///   [`Sampler::unconstrained`] but the *same* constrained `checker` —
    ///   this is exactly what [`RrtConnectContext::solve`] would do if the
    ///   round 20 wiring did not exist. Must fail within the budget.
    /// - **Wired**: the exact same query through the real
    ///   [`RrtConnectManager::get_planning_context`] -> `solve()` path,
    ///   same seed, same budget. Must succeed, and every waypoint's
    ///   `panda_joint1` must sit inside the window — proving the registry's
    ///   sampler wiring, not the checker alone, is what turns a budget the
    ///   checker-only search exhausts into one the search solves within.
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

        let joint_constraint = JointConstraint::new(&model, "panda_joint1", 0.0, 0.005, 0.005, 1.0)
            .expect("valid joint constraint");
        let mut path_constraints = KinematicConstraintSet::new();
        path_constraints.push(Constraint::Joint(joint_constraint));

        let small_budget = RrtConnectParams {
            step_size: 0.5,
            goal_bias: 0.0,
            termination: Termination::Iterations(20),
            nn_degree: 8,
        };

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
            Sampler::unconstrained(&mut ChaCha8Rng::seed_from_u64(3)),
            &small_budget,
        );
        assert_eq!(
            control_result,
            Err(PlanningFailure::IterationsExhausted),
            "the unwired control (checker-only, no sampler) must NOT find the path the wired \
             search below finds within the same iteration budget"
        );

        // Wired: the real registry path.
        let mut scene = PlanningScene::new(&model, &srdf);
        let env = ParryCollisionEnv::default();
        let manager = RrtConnectManager;
        let request = PlanningRequest {
            group_name: "panda_arm".to_string(),
            goal,
            path_constraints: Some(path_constraints),
            resolution: 0.05,
            seed: 3,
            params: small_budget,
        };
        let mut context = manager
            .get_planning_context(&mut scene, &env, request)
            .expect("panda_arm is a real group");
        let response = context.solve().expect(
            "the wired search must solve within the same iteration budget the unwired control \
             above exhausts",
        );
        drop(context);

        for (index, waypoint) in response.trajectory.iter().enumerate() {
            let value = waypoint.variable_position("panda_joint1").unwrap();
            assert!(
                (-0.005..=0.005).contains(&value),
                "waypoint {index}: panda_joint1 = {value} escaped the +/-0.005 constraint window"
            );
        }
    }
}
