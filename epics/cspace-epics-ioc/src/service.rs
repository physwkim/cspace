//! The EPICS-agnostic planning core: owns the robot model loaded at IOC
//! startup and answers "start joints + goal joints → a timed joint
//! trajectory".
//!
//! This is the same layering `ros/cspace-ros`'s `/move_action` server uses
//! — [`cspace_planning::planner_registry::resolve_planner`] on top of
//! [`cspace_planning::generate_plan`] — with one deliberate difference:
//! the response chain carries [`AddTimeOptimalParameterization`] (TOTG),
//! so the trajectory published over Channel Access has real per-waypoint
//! timestamps instead of the all-zero durations a bare planner emits
//! (cspace-ros documents its empty chain as a gap, not a decision).
//!
//! TOTG refuses any group variable without an acceleration bound, and the
//! URDF `<limit>` element has no acceleration attribute at all — MoveIt
//! sources those limits from `joint_limits.yaml`, a file this IOC does not
//! read. [`PlannerService::from_files`] therefore seeds
//! [`DEFAULT_MAX_ACCELERATION`] into every group variable the URDF left
//! acceleration-unbounded, once at load. Ruckig smoothing is NOT a
//! substitute for TOTG here: the Ruckig port (like upstream) only rewrites
//! the final segment's duration and inherits every other segment's timing
//! from its input, so on a planner's all-zero-duration output it yields
//! `[0, 0, …, D]` — Ruckig smooths an already-timed trajectory, it does
//! not create timing (measured before this seeding existed).

use std::path::{Path, PathBuf};

use cspace_collision::ParryCollisionEnv;
use cspace_core::model::joint::VariableBounds;
use cspace_core::model::{MeshSearchPaths, RobotModel};
use cspace_core::srdf::SrdfModel;
use cspace_core::state::RobotState;
use cspace_planning::constraints::utils::construct_goal_joint_constraints;
use cspace_planning::planner_registry::resolve_planner;
use cspace_planning::response_adapters::AddTimeOptimalParameterization;
use cspace_planning::scene::PlanningScene;
use cspace_planning::{
    PipelineError, PlannerConfigurationMap, PlanningRequest, PlanningResponseAdapter, StartState,
    generate_plan,
};

/// The planner used when the planner-id PV is empty. Mirrors
/// `cspace-ros`'s `DEFAULT_PIPELINE_ID`: an explicit named default, never
/// "the first registry entry" (`linkme` slice order is the linker's).
pub const DEFAULT_PLANNER_ID: &str = "rrt_connect";

/// Acceleration bound, rad/s², seeded into every group variable the URDF
/// leaves acceleration-unbounded (URDF cannot express one; see the module
/// doc). The figure is the same default the Ruckig port substitutes for a
/// missing bound; panda's real `joint_limits.yaml` values span 3.75–15.
pub const DEFAULT_MAX_ACCELERATION: f64 = 10.0;

/// Maps a scaling-factor PV onto the pipeline's accepted range: anything
/// outside `(0, 1]` (an untouched `0.0` PV included) means "full speed".
/// TOTG's `verify_scaling_factor` does the same replacement internally,
/// but keeping the rule here makes the PV contract ("outside (0,1] →
/// 1.0") independent of which response adapters the chain carries.
fn normalized_scale(factor: f64) -> f64 {
    if factor > 0.0 && factor <= 1.0 {
        factor
    } else {
        1.0
    }
}

/// Gives every variable of `group_name`'s joints an acceleration bound of
/// [`DEFAULT_MAX_ACCELERATION`] unless the model already has one, so TOTG
/// can time-parameterize (see the module doc for why the URDF never
/// provides these). Velocity bounds are untouched — URDF does carry those.
fn seed_missing_acceleration_bounds(
    model: &mut RobotModel,
    group_name: &str,
) -> Result<(), cspace_core::error::Error> {
    let group = model.joint_model_group(group_name)?;
    let mut patches: Vec<(String, String, VariableBounds)> = Vec::new();
    for &joint_index in group.joint_indices() {
        let joint = model.joint_model_at(joint_index);
        for (variable, bounds) in joint.variable_names().iter().zip(joint.variable_bounds()) {
            if !bounds.acceleration_bounded {
                patches.push((
                    joint.name().to_string(),
                    variable.clone(),
                    VariableBounds {
                        min_acceleration: -DEFAULT_MAX_ACCELERATION,
                        max_acceleration: DEFAULT_MAX_ACCELERATION,
                        acceleration_bounded: true,
                        ..*bounds
                    },
                ));
            }
        }
    }
    for (joint, variable, bounds) in patches {
        model
            .joint_model_mut(&joint)?
            .set_variable_bounds(&variable, bounds)?;
    }
    Ok(())
}

/// Everything that can go wrong loading the model or answering a plan.
#[derive(Debug, thiserror::Error)]
pub enum ServiceError {
    /// A URDF/SRDF file could not be read.
    #[error("failed to read {path}: {source}")]
    Io {
        /// The file that could not be read.
        path: PathBuf,
        /// The underlying I/O error.
        #[source]
        source: std::io::Error,
    },
    /// The URDF file did not parse.
    #[error("failed to parse URDF {path}: {source}")]
    Urdf {
        /// The file that did not parse.
        path: PathBuf,
        /// The underlying parse error.
        #[source]
        source: urdf_rs::UrdfError,
    },
    /// Model construction, group lookup, constraint building, planner
    /// resolution, or trajectory extraction failed in a core crate.
    #[error(transparent)]
    Core(#[from] cspace_core::error::Error),
    /// The start array's length does not match the planning group.
    #[error("start has {got} values, group '{group}' has {expected} joints")]
    StartJointCount {
        /// Values received.
        got: usize,
        /// Joints in the group.
        expected: usize,
        /// The planning group.
        group: String,
    },
    /// The goal array's length does not match the planning group.
    #[error("goal has {got} values, group '{group}' has {expected} joints")]
    GoalJointCount {
        /// Values received.
        got: usize,
        /// Joints in the group.
        expected: usize,
        /// The planning group.
        group: String,
    },
    /// The planning pipeline itself failed (adapters or planner).
    #[error(transparent)]
    Pipeline(#[from] PipelineError),
}

/// An owned, EPICS-shaped extraction of a [`cspace_planning::PlanningResponse`]:
/// no borrow of the robot model escapes the service.
#[derive(Debug, Clone)]
pub struct PlannedTrajectory {
    /// The group's joint (variable) names, in the column order of
    /// [`Self::positions`].
    pub joint_names: Vec<String>,
    /// Number of waypoints.
    pub n_points: usize,
    /// Row-major `[n_points][joint_names.len()]` joint positions.
    pub positions: Vec<f64>,
    /// Seconds from trajectory start, one entry per waypoint.
    pub times_from_start: Vec<f64>,
}

/// Owns the robot model and planning scene inputs loaded once at IOC
/// startup (`CspacePlannerConfig`); [`Self::plan`] builds a fresh scene and
/// collision environment per request, the same per-plan pattern
/// `cspace-ros`'s `/move_action` node uses.
pub struct PlannerService {
    model: RobotModel,
    srdf: SrdfModel,
    group_name: String,
    joint_names: Vec<String>,
}

impl PlannerService {
    /// Loads URDF + SRDF from disk and resolves `group_name` against the
    /// model. `mesh_search_paths` maps `package://` names to directories
    /// for collision meshes; empty means no collision geometry is loaded
    /// (mesh-referencing `<collision>` elements are skipped).
    pub fn from_files(
        urdf_path: &Path,
        srdf_path: &Path,
        group_name: &str,
        mesh_search_paths: &[(String, PathBuf)],
    ) -> Result<Self, ServiceError> {
        let urdf_xml = std::fs::read_to_string(urdf_path).map_err(|source| ServiceError::Io {
            path: urdf_path.to_path_buf(),
            source,
        })?;
        let urdf = urdf_rs::read_from_string(&urdf_xml).map_err(|source| ServiceError::Urdf {
            path: urdf_path.to_path_buf(),
            source,
        })?;
        let srdf = SrdfModel::parse_file(srdf_path)?;
        let meshes = if mesh_search_paths.is_empty() {
            MeshSearchPaths::none()
        } else {
            MeshSearchPaths::new(
                mesh_search_paths
                    .iter()
                    .map(|(package, dir)| (package.clone(), dir.clone())),
            )
        };
        let mut model = RobotModel::from_urdf_and_srdf(&urdf, &urdf_xml, &srdf, &meshes)?;
        seed_missing_acceleration_bounds(&mut model, group_name)?;
        let joint_names = model
            .joint_model_group(group_name)?
            .variable_names()
            .to_vec();
        Ok(Self {
            model,
            srdf,
            group_name: group_name.to_string(),
            joint_names,
        })
    }

    /// The group's joint (variable) names, in the order [`Self::plan`]
    /// expects `start`/`goal` values and emits trajectory columns.
    pub fn joint_names(&self) -> &[String] {
        &self.joint_names
    }

    /// Plans from `start` to `goal` (both in [`Self::joint_names`] order)
    /// and time-parameterizes the result with TOTG.
    ///
    /// An empty `planner_id` selects [`DEFAULT_PLANNER_ID`]. Scaling
    /// factors outside `(0, 1]` are replaced by `1.0` ([`normalized_scale`]),
    /// so an untouched scale PV of `0.0` means "full speed".
    pub fn plan(
        &self,
        start: &[f64],
        goal: &[f64],
        planner_id: &str,
        max_velocity_scaling_factor: f64,
        max_acceleration_scaling_factor: f64,
    ) -> Result<PlannedTrajectory, ServiceError> {
        let expected = self.joint_names.len();
        if start.len() != expected {
            return Err(ServiceError::StartJointCount {
                got: start.len(),
                expected,
                group: self.group_name.clone(),
            });
        }
        if goal.len() != expected {
            return Err(ServiceError::GoalJointCount {
                got: goal.len(),
                expected,
                group: self.group_name.clone(),
            });
        }

        let mut scene = PlanningScene::new(&self.model, &self.srdf);
        let env = ParryCollisionEnv::new(scene.world().clone(), Default::default());

        let mut goal_state = RobotState::new(&self.model);
        goal_state.set_to_default_values();
        goal_state.set_joint_group_positions(&self.group_name, goal)?;
        // Zero-width tolerances on purpose: RRT-Connect samples the goal
        // region, and only a zero-width window makes the draw reproduce the
        // requested value (cspace-ros's move_group test documents this).
        let goal_constraints = construct_goal_joint_constraints(
            &self.model,
            &goal_state.update(),
            &self.group_name,
            0.0,
            0.0,
        )?;

        let planner_name = if planner_id.is_empty() {
            DEFAULT_PLANNER_ID
        } else {
            planner_id
        };
        let planner = resolve_planner(planner_name, &PlannerConfigurationMap::new())?;

        let request = PlanningRequest {
            group_name: self.group_name.clone(),
            start_state: StartState::new(self.joint_names.clone(), start.to_vec(), vec![])?,
            goal_constraints: vec![goal_constraints],
            max_velocity_scaling_factor: normalized_scale(max_velocity_scaling_factor),
            max_acceleration_scaling_factor: normalized_scale(max_acceleration_scaling_factor),
            planner_id: planner_name.to_string(),
            ..PlanningRequest::default()
        };

        // Arguments match `TotgOptions::default()` (upstream's own
        // constructor defaults); the adapter has no `Default` impl.
        let response_chain: Vec<Box<dyn PlanningResponseAdapter>> = vec![Box::new(
            AddTimeOptimalParameterization::new(0.1, 0.1, 0.001),
        )];
        let response = generate_plan(&mut scene, &env, &[], &[planner], &response_chain, request)?;

        let n_points = response.trajectory.way_point_count();
        let mut positions = Vec::with_capacity(n_points * expected);
        let mut times_from_start = Vec::with_capacity(n_points);
        for i in 0..n_points {
            let way_point = response.trajectory.way_point(i)?;
            positions.extend(way_point.joint_group_positions(&self.group_name)?);
            times_from_start.push(response.trajectory.way_point_duration_from_start(i));
        }
        Ok(PlannedTrajectory {
            joint_names: self.joint_names.clone(),
            n_points,
            positions,
            times_from_start,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn panda() -> PlannerService {
        let root = concat!(env!("CARGO_MANIFEST_DIR"), "/../../fixtures");
        PlannerService::from_files(
            Path::new(&format!("{root}/panda.urdf")),
            Path::new(&format!("{root}/panda.srdf")),
            "panda_arm",
            &[],
        )
        .expect("the panda fixture must load")
    }

    /// Within-limits start pose for the panda arm (its "ready" pose):
    /// joint 4's limit range is entirely negative, so all-zeros is not a
    /// valid state.
    const READY: [f64; 7] = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785];

    #[test]
    fn plans_between_two_joint_states() {
        let service = panda();
        let mut goal = READY;
        goal[0] = 0.5;
        let planned = service
            .plan(&READY, &goal, "", 1.0, 1.0)
            .expect("a small joint-space move must plan");
        let n = planned.joint_names.len();
        assert_eq!(n, 7);
        assert!(planned.n_points >= 2);
        assert_eq!(planned.positions.len(), planned.n_points * n);
        assert_eq!(planned.times_from_start.len(), planned.n_points);
        for (a, b) in planned.positions[..n].iter().zip(READY) {
            assert!((a - b).abs() < 1e-6, "first waypoint must be the start");
        }
        for (a, b) in planned.positions[planned.positions.len() - n..]
            .iter()
            .zip(goal)
        {
            assert!((a - b).abs() < 1e-6, "last waypoint must be the goal");
        }
        assert_eq!(planned.times_from_start[0], 0.0);
        assert!(
            planned.times_from_start[planned.n_points - 1] > 0.0,
            "TOTG must produce nonzero timing"
        );
        for pair in planned.times_from_start.windows(2) {
            assert!(pair[0] < pair[1], "times must be strictly increasing");
        }
    }

    #[test]
    fn start_of_wrong_length_is_rejected() {
        let service = panda();
        let err = service
            .plan(&[0.0; 3], &READY, "", 1.0, 1.0)
            .expect_err("3 values against a 7-joint group must fail");
        assert!(matches!(
            err,
            ServiceError::StartJointCount {
                got: 3,
                expected: 7,
                ..
            }
        ));
    }

    #[test]
    fn goal_of_wrong_length_is_rejected() {
        let service = panda();
        let err = service
            .plan(&READY, &[0.0; 9], "", 1.0, 1.0)
            .expect_err("9 values against a 7-joint group must fail");
        assert!(matches!(
            err,
            ServiceError::GoalJointCount {
                got: 9,
                expected: 7,
                ..
            }
        ));
    }

    #[test]
    fn unknown_planner_id_is_rejected() {
        let service = panda();
        let err = service
            .plan(&READY, &READY, "no_such_planner", 1.0, 1.0)
            .expect_err("an unregistered planner id must fail");
        assert!(matches!(err, ServiceError::Core(_)));
    }
}
