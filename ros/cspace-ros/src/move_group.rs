// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause

//! `MoveGroupMoveAction`'s plan-only arm, as far as it is expressible against
//! this workspace's own planning types.
//!
//! Two upstream functions land here, both from
//! `moveit_ros/move_group/src/`:
//!
//! * `MoveGroupCapability::resolvePlanningPipeline`
//!   (`move_group_capability.cpp:223-246`) — [`resolve_planning_pipeline`].
//! * `MoveGroupMoveAction::executeMoveCallbackPlanOnly`'s planning body
//!   (`default_capabilities/move_action_capability.cpp:206-227`) —
//!   [`plan_only`].
//!
//! They are here, in the library, rather than inline in
//! `src/bin/move_group.rs`, because a `[[bin]]`'s functions are reachable
//! only from that binary: the node's own goal handler and this module's tests
//! have to be calling the same code for the tests to say anything about the
//! node.
//!
//! # What "pipeline" means here
//!
//! Upstream a *pipeline* is a `planning_pipeline::PlanningPipeline`: one
//! `planning_interface::PlannerManager` plugin loaded by pluginlib from the
//! pipeline's own `planning_plugin` parameter, plus that pipeline's adapter
//! chains. This port has no plugin loader and no parameter server; the
//! equivalent lookup is [`cspace_planner_registry::resolve_planner`], keyed
//! by [`cspace_planning::PlannerManager::name`] (PORTING-PLAN.md D8/§140).
//! So `pipeline_id` selects a *planner manager* here, not a pipeline object,
//! and the two coincide only because every registered manager in this
//! workspace is its own whole pipeline.
//!
//! # The adapter chains are empty, and that is a gap, not a decision
//!
//! [`plan_only`] passes `&[]` for both of
//! [`cspace_planning::generate_plan`]'s adapter chains. Upstream builds
//! those from the pipeline's `request_adapters`/`response_adapters`
//! parameters, and a default `move_group` launch configures several
//! (`ValidateWorkspaceBounds`, `CheckStartStateBounds`,
//! `AddTimeOptimalParameterization`, …). Nothing in this crate reads
//! parameters yet, so there is no configured chain to pass and no honest way
//! to invent one; the consequence is that a plan produced through this
//! module carries no time parameterization and has had no start-state
//! validation beyond what the planner itself does. That ends when this crate
//! grows parameter handling, not when someone hardcodes a list here.

use std::fmt;

use cspace_collision::ParryCollisionEnv;
use cspace_planner_registry::resolve_planner;
use cspace_planning::{
    PipelineError, PlannerConfigurationMap, PlannerManager, PlanningRequest, PlanningResponse,
};
use cspace_scene::PlanningScene;

/// The planner an empty `pipeline_id` resolves to.
///
/// Upstream's empty-`pipeline_id` branch returns `context_->planning_pipeline_`
/// (`move_group_capability.cpp:225-229`), the pipeline `move_group` was
/// launched with — a value from configuration, not a name in the source.
/// This crate has no configuration to read it from, and
/// `cspace_planner_registry::PLANNER_MANAGERS` is deliberately unordered
/// (PORTING-PLAN.md §177: link-section order is not a contract), so "the
/// first registration" is not a definition either. Naming the default
/// explicitly is the only remaining option that gives the same answer twice.
pub const DEFAULT_PIPELINE_ID: &str = "rrt_connect";

/// `MoveGroupCapability::resolvePlanningPipeline`
/// (`move_group_capability.cpp:223-246`).
///
/// An empty `pipeline_id` resolves to [`DEFAULT_PIPELINE_ID`]; any other
/// value is looked up by name. `None` is upstream's null
/// `PlanningPipelinePtr` return — the caller's cue to report
/// `MoveItErrorCodes::FAILURE` (`move_action_capability.cpp:207-211`).
///
/// Upstream logs the miss (`RCLCPP_WARN "Couldn't find requested planning
/// pipeline '%s'"`); this returns it instead, because the one caller
/// [`plan_only`] puts the name into the error it reports and a log line
/// would say it to a different audience.
///
/// `configs` is the node's `/set_planner_params` store, and the manager is
/// *built from* it — upstream's `setPlannerConfigurations` on the resolved
/// pipeline (`query_planners_service_capability.cpp:205`) has no counterpart
/// call here because the configuration is a constructor argument instead
/// (PORTING-PLAN.md §285). A caller passing an empty map gets a planner in
/// its own documented defaults, which is what every non-ROS caller wants.
pub fn resolve_planning_pipeline(
    pipeline_id: &str,
    configs: &PlannerConfigurationMap,
) -> Option<Box<dyn PlannerManager>> {
    let name = if pipeline_id.is_empty() {
        DEFAULT_PIPELINE_ID
    } else {
        pipeline_id
    };
    resolve_planner(name, configs).ok()
}

/// Why a [`plan_only`] call produced no trajectory.
///
/// Both variants are `MoveItErrorCodes::FAILURE` upstream — the unresolved
/// pipeline at `move_action_capability.cpp:207-211` and the failed/throwing
/// `generatePlan` at `:215-227` — so this enum does not exist to pick
/// different error codes. It exists so the message that goes back over the
/// wire can say which of the two happened; upstream distinguishes them only
/// in its log.
#[derive(Debug)]
pub enum PlanOnlyError {
    /// `resolvePlanningPipeline` returned null: no planner is registered
    /// under this `pipeline_id`.
    UnknownPipeline {
        /// The `pipeline_id` as it arrived on the wire, empty string
        /// included (an empty one still fails here if
        /// [`DEFAULT_PIPELINE_ID`] itself is unregistered).
        pipeline_id: String,
    },
    /// The planner resolved and ran, and the plan did not come out.
    Planning(PipelineError),
}

impl fmt::Display for PlanOnlyError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::UnknownPipeline { pipeline_id } => {
                write!(f, "no planner is registered as '{pipeline_id}'")
            }
            Self::Planning(source) => write!(f, "planning failed: {source}"),
        }
    }
}

impl std::error::Error for PlanOnlyError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Self::UnknownPipeline { .. } => None,
            Self::Planning(source) => Some(source),
        }
    }
}

/// `MoveGroupMoveAction::executeMoveCallbackPlanOnly`'s planning body
/// (`move_action_capability.cpp:206-227`): resolve the pipeline named by the
/// request, then hand it the scene and the request.
///
/// Upstream's `catch (std::exception&)` around `generatePlan` has no
/// counterpart: [`cspace_planning::generate_plan`] returns its failures, and
/// a Rust panic is not something this function could turn into a
/// `MoveItErrorCodes` without lying about what happened.
///
/// The scene is `&mut` and stays mutated: upstream plans against a *copy*
/// (`planning_scene_monitor_->copyPlanningScene(diff)`, `:216-217`), so a
/// planner that moves the current state cannot affect the next goal. The
/// isolation is the caller's to provide here, and `src/bin/move_group.rs`
/// provides it the way upstream does — [`cspace_scene::PlanningScene::diff`]
/// off the monitored snapshot, so what this function mutates is the child.
/// A caller that hands over the monitored scene itself gets no such
/// separation, which is why the parameter is `&mut` rather than `&`: the
/// type says the scene is written to.
pub fn plan_only<'m>(
    scene: &mut PlanningScene<'m>,
    env: &ParryCollisionEnv,
    pipeline_id: &str,
    configs: &PlannerConfigurationMap,
    request: PlanningRequest,
) -> Result<PlanningResponse<'m>, PlanOnlyError> {
    let Some(planner) = resolve_planning_pipeline(pipeline_id, configs) else {
        return Err(PlanOnlyError::UnknownPipeline {
            pipeline_id: pipeline_id.to_string(),
        });
    };
    cspace_planning::generate_plan(scene, env, &[], &[planner], &[], request)
        .map_err(PlanOnlyError::Planning)
}

#[cfg(test)]
mod tests {
    use cspace_constraints::utils::construct_goal_joint_constraints;
    use cspace_model::{MeshSearchPaths, RobotModel};
    use cspace_srdf::SrdfModel;
    use cspace_state::RobotState;

    use super::*;

    const ONE_JOINT_URDF: &str = r#"<?xml version="1.0"?>
<robot name="one_joint">
  <link name="base_link"/>
  <link name="tip"/>
  <joint name="j1" type="revolute">
    <parent link="base_link"/>
    <child link="tip"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1" upper="1" effort="10" velocity="1"/>
  </joint>
</robot>
"#;

    const ONE_JOINT_SRDF: &str = r#"<?xml version="1.0"?>
<robot name="one_joint">
  <group name="arm">
    <chain base_link="base_link" tip_link="tip"/>
  </group>
</robot>
"#;

    /// `ros/fixtures/one_joint.{urdf,srdf}`, inline — the same robot both
    /// live legs of `ros/verify-ros-interop.sh` load, so a plan that
    /// succeeds here is a plan for the robot the node is actually serving.
    fn one_joint() -> (RobotModel, SrdfModel) {
        let urdf = urdf_rs::read_from_string(ONE_JOINT_URDF).expect("inline URDF must parse");
        let srdf = SrdfModel::parse_str(ONE_JOINT_SRDF).expect("inline SRDF must parse");
        let model =
            RobotModel::from_urdf_and_srdf(&urdf, ONE_JOINT_URDF, &srdf, &MeshSearchPaths::none())
                .expect("valid single-joint urdf");
        (model, srdf)
    }

    /// A goal in the shape upstream builds one,
    /// `constructGoalConstraints(state, jmg, tolerance)`
    /// (`moveit_core/kinematic_constraints/include/moveit/kinematic_constraints/utils.hpp:99`):
    /// one `JointConstraint` per group variable at `j1 == position`.
    ///
    /// Zero rather than upstream's `numeric_limits<double>::epsilon()`
    /// default: `RrtConnectManager` *samples* its goal region, and only a
    /// zero-width window makes that draw reproduce `position` itself rather
    /// than a neighbour of it (see `construct_goal_joint_constraints`' own
    /// doc). A goal a client names by joint value is a request to go there,
    /// not near there.
    fn goal_at(model: &RobotModel, position: f64) -> cspace_constraints::KinematicConstraintSet {
        let mut state = RobotState::new(model);
        state.set_to_default_values();
        state
            .set_variable_position("j1", position)
            .expect("j1 is one_joint.urdf's only joint");
        let posed = state.update();
        construct_goal_joint_constraints(model, &posed, "arm", 0.0, 0.0)
            .expect("arm is one_joint.srdf's only group")
    }

    fn request_for(model: &RobotModel, position: f64) -> PlanningRequest {
        PlanningRequest {
            group_name: "arm".to_string(),
            goal_constraints: vec![goal_at(model, position)],
            ..PlanningRequest::default()
        }
    }

    /// The round's success criterion: the plan-only arm reaches a real
    /// planner and gets a real trajectory back. Everything else in this
    /// module is about which failure gets reported; this is the one case
    /// where nothing fails.
    ///
    /// `planner_id` is asserted because it is the only field of the response
    /// that names *which* planner ran — a trajectory alone would be
    /// identical if some other registration had been picked up.
    #[test]
    fn the_plan_only_arm_reaches_rrt_connect_and_gets_a_trajectory() {
        let (model, srdf) = one_joint();
        let mut scene = PlanningScene::new(&model, &srdf);
        let env = ParryCollisionEnv::default();

        let response = plan_only(
            &mut scene,
            &env,
            "",
            &PlannerConfigurationMap::new(),
            request_for(&model, 0.5),
        )
        .expect("one_joint's `arm` group has a reachable goal at j1 = 0.5");

        assert_eq!(
            response.planner_id, "rrt_connect",
            "the plan must come from the planner the registry resolved, named by that planner"
        );
        assert!(
            response.trajectory.way_point_count() >= 2,
            "a plan from a start that is not already the goal has at least a start and an end, \
             got {} waypoint(s)",
            response.trajectory.way_point_count()
        );
        let last = response
            .trajectory
            .way_point(response.trajectory.way_point_count() - 1)
            .expect("the index is one below the count");
        let reached = last
            .variable_position("j1")
            .expect("j1 is one_joint.urdf's only joint");
        assert!(
            (reached - 0.5).abs() < 1e-6,
            "the last waypoint must be inside the requested goal region, got j1 = {reached}"
        );
    }

    /// The store `planner_params::spawn` hands back reaches the planner:
    /// the same query, planned twice through [`plan_only`], differs when a
    /// `range` configuration is in the map.
    ///
    /// This is the node-side half of PORTING-PLAN.md §285 — the sbp crate's
    /// `a_range_configuration_reaches_the_registry_planner_and_changes_the_plan`
    /// proves the registry hands the map on, and this proves the function
    /// the node's goal handler actually calls carries it that far.
    ///
    /// Keyed `"arm"`, the bare group name: `request_for` leaves `planner_id`
    /// empty, which is what an unmodified `MoveGroupInterface` client sends,
    /// and an empty `planner_id` skips straight to
    /// `cspace_planning::configuration_for`'s group-default lookup. That is
    /// the key `/set_planner_params` writes for
    /// `{group: "", planner_config: "arm"}`.
    #[test]
    fn a_stored_configuration_changes_the_plan_plan_only_produces() {
        let (model, srdf) = one_joint();
        let env = ParryCollisionEnv::default();

        let plan_with = |configs: &PlannerConfigurationMap| {
            let mut scene = PlanningScene::new(&model, &srdf);
            let response = plan_only(&mut scene, &env, "", configs, request_for(&model, 0.5))
                .expect("one_joint's `arm` group has a reachable goal at j1 = 0.5");
            (0..response.trajectory.way_point_count())
                .map(|i| {
                    response
                        .trajectory
                        .way_point(i)
                        .expect("the index is below the count")
                        .variable_position("j1")
                        .expect("j1 is one_joint.urdf's only joint")
                })
                .collect::<Vec<f64>>()
        };

        let name = cspace_planning::configuration_name("", "arm");
        let mut configured = PlannerConfigurationMap::new();
        configured.insert(
            name.clone(),
            cspace_planning::PlannerConfigurationSettings {
                group: String::new(),
                name,
                config: [("range".to_string(), "0.05".to_string())]
                    .into_iter()
                    .collect(),
            },
        );

        let unconfigured_path = plan_with(&PlannerConfigurationMap::new());
        let configured_path = plan_with(&configured);
        assert_ne!(
            unconfigured_path, configured_path,
            "a configuration in the node's store must change the plan; identical paths mean \
             `plan_only` dropped the map on the way to the planner"
        );
        assert!(
            configured_path.len() > unconfigured_path.len(),
            "`range` caps one extend, so a tenth of the default must take more of them: \
             got {} waypoint(s) configured against {} unconfigured",
            configured_path.len(),
            unconfigured_path.len()
        );
        for (label, path) in [
            ("unconfigured", &unconfigured_path),
            ("configured", &configured_path),
        ] {
            let reached = *path.last().expect("a solved plan has waypoints");
            assert!(
                (reached - 0.5).abs() < 1e-6,
                "the {label} plan must still end inside the goal region, got j1 = {reached}"
            );
        }
    }

    /// The `pipeline_id == ""` boundary of [`resolve_planning_pipeline`]:
    /// upstream's empty branch returns the configured default rather than
    /// looking anything up (`move_group_capability.cpp:225-229`), and this
    /// port's stand-in for "configured" is [`DEFAULT_PIPELINE_ID`]. A miss
    /// here means an unnamed pipeline (which is what every
    /// `MoveGroupInterface` client sends unless told otherwise) resolves to
    /// nothing.
    #[test]
    fn an_empty_pipeline_id_resolves_to_the_named_default() {
        let planner = resolve_planning_pipeline("", &PlannerConfigurationMap::new())
            .expect("an empty pipeline_id must reach the default");
        assert_eq!(planner.name(), DEFAULT_PIPELINE_ID);
    }

    /// The other side of that boundary: a non-empty `pipeline_id` is looked
    /// up verbatim, never silently replaced by the default. If the `is_empty`
    /// test above were inverted or dropped, this is the case that notices.
    #[test]
    fn a_named_pipeline_id_is_looked_up_verbatim() {
        let planner = resolve_planning_pipeline("rrt_connect", &PlannerConfigurationMap::new())
            .expect("rrt_connect is registered under its own name");
        assert_eq!(planner.name(), "rrt_connect");
        assert!(
            resolve_planning_pipeline("ompl", &PlannerConfigurationMap::new()).is_none(),
            "a pipeline_id naming no registered planner must not fall back to the default"
        );
    }

    /// [`PlanOnlyError::UnknownPipeline`], the first of
    /// `executeMoveCallbackPlanOnly`'s two `FAILURE` sites (`:207-211`).
    /// The unresolved name has to survive into the error, since that is the
    /// only thing distinguishing this failure from the other one for a
    /// caller reading the message off the wire.
    #[test]
    fn an_unregistered_pipeline_id_fails_before_any_planning_runs() {
        let (model, srdf) = one_joint();
        let mut scene = PlanningScene::new(&model, &srdf);
        let env = ParryCollisionEnv::default();

        // `let Err(..) else` rather than `expect_err`: the `Ok` type is
        // `PlanningResponse`, whose `Debug` carries a full `RobotModel` per
        // waypoint, so a regression here would bury its own message under
        // ~10 KB of fixture (measured while biting this test).
        let Err(err) = plan_only(
            &mut scene,
            &env,
            "ompl",
            &PlannerConfigurationMap::new(),
            request_for(&model, 0.5),
        ) else {
            panic!("no planner is registered as 'ompl'");
        };
        match err {
            PlanOnlyError::UnknownPipeline { pipeline_id } => assert_eq!(pipeline_id, "ompl"),
            PlanOnlyError::Planning(source) => {
                panic!("expected UnknownPipeline, got a planning failure: {source}")
            }
        }
    }

    /// [`PlanOnlyError::Planning`], the second `FAILURE` site (`:215-227`):
    /// the pipeline resolved, ran, and did not produce a plan. Distinct from
    /// the case above in *which* variant comes back, which is what the
    /// wire-side message is built from.
    ///
    /// `not_a_group` rather than an unreachable goal: an unreachable goal
    /// makes RRT-Connect run its whole iteration budget before failing, and
    /// a rejected group name reaches the same variant in microseconds.
    #[test]
    fn a_planner_that_rejects_the_request_fails_as_a_planning_failure() {
        let (model, srdf) = one_joint();
        let mut scene = PlanningScene::new(&model, &srdf);
        let env = ParryCollisionEnv::default();

        let request = PlanningRequest {
            group_name: "not_a_group".to_string(),
            ..request_for(&model, 0.5)
        };
        let Err(err) = plan_only(
            &mut scene,
            &env,
            "",
            &PlannerConfigurationMap::new(),
            request,
        ) else {
            panic!("one_joint.srdf has no group called 'not_a_group'");
        };
        match err {
            PlanOnlyError::Planning(PipelineError::Planner { planner, .. }) => {
                assert_eq!(
                    planner, "rrt_connect",
                    "the failure must be attributed to the planner that actually ran"
                );
            }
            other => panic!("expected Planning(PipelineError::Planner), got {other:?}"),
        }
    }
}
