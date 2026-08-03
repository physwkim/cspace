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
//! `constraint_samplers` sampler. Per `PORTING-PLAN.md` D1 (no
//! `moveit_msgs`) this port has no `moveit_msgs::msg::Constraints` to carry,
//! and — independently of D1 — no `constraint_samplers` equivalent exists
//! anywhere in this workspace to turn one into states (`PORTING-PLAN.md`
//! §3 nominally scopes `constraint_samplers` under `moveit-constraints`,
//! but it was never ported there).
//!
//! [`PlanningRequest::goal`] is therefore a concrete
//! [`crate::joint_model_group_space::JointModelGroupSpace`] state
//! (`Vec<CompoundValue>`), not a constraint to sample from. The rejected
//! alternative was a single-[`moveit_constraints::JointConstraint`]-per-variable
//! stub standing in for a real sampler: it would silently mishandle any
//! Cartesian (position/orientation) goal, which is exactly the case a real
//! sampler exists to handle, so a stub narrow enough to always work (joint
//! goals only) would be indistinguishable from this concrete-state design
//! except for an extra, misleading layer of indirection. [`PlanningRequest::path_constraints`]
//! *is* carried directly as a [`KinematicConstraintSet`], because path
//! constraints are evaluated per-candidate via `decide()` — see
//! [`crate::planning_scene_validity::PlanningSceneValidityChecker`] — never
//! sampled from, so D1's missing sampler does not block them.
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
use moveit_constraints::KinematicConstraintSet;
use moveit_scene::PlanningScene;
use moveit_state::RobotState;
use rand::SeedableRng;
use rand_chacha::ChaCha8Rng;

use crate::compound::CompoundValue;
use crate::error::SbpError;
use crate::joint_model_group_space::JointModelGroupSpace;
use crate::planning_scene_validity::PlanningSceneValidityChecker;
use crate::rrt_connect::{PlanningFailure, RrtConnectParams, rrt_connect};
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
    /// in the group's own [`StateSpace::distance`] units.
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
            &mut rng,
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
        let mut rng = ChaCha8Rng::seed_from_u64(7);
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
            seed: 11,
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
}
