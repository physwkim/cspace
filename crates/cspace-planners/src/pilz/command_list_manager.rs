// Copyright (c) 2018, Pilz GmbH & Co. KG
// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// Ported from moveit2 @ e017c91ee12984393a28ba246075c65f69cde3bf:
//   moveit_planners/pilz_industrial_motion_planner/include/pilz_industrial_motion_planner/command_list_manager.hpp
//   moveit_planners/pilz_industrial_motion_planner/src/command_list_manager.cpp

//! Planning of a *sequence* of motion commands ([`CommandListManager`]).
//!
//! Upstream `CommandListManager`. A sequence is a list of
//! [`SequenceItem`]s, each a motion request plus the blend radius joining it
//! to the *next* item. [`CommandListManager::solve`] validates the list,
//! plans each item with the previous item's end state as its start state,
//! and assembles the results through
//! [`PlanComponentsBuilder`].
//!
//! The rules upstream's `solve` doc states, unchanged here:
//!
//! - two consecutive trajectories of the same group with a zero blend radius
//!   are concatenated;
//! - the same group with a positive radius are blended;
//! - different groups start a new element of the result list.
//!
//! And the four conditions a valid list must satisfy, each with its own
//! [`SequenceError`] variant: every radius non-negative, the last radius
//! zero, only the first request of each group carrying a start state, and no
//! two consecutive radii overlapping.
//!
//! # Deviations from upstream
//!
//! - **The per-item planner is the caller's.** Upstream `solve` takes a
//!   `planning_pipeline::PlanningPipelinePtr` and calls `generatePlan`, which
//!   selects a generator from `req.planner_id` through the
//!   `planning_context_loader_*` `pluginlib` plugins — the layer this crate
//!   excludes under `D1`/`D2` (see the crate docs). [`CommandListManager::
//!   solve`] takes a `solve_one` closure instead, so the caller binds the
//!   generator and the sampling time. `CommandListManager` never knew which
//!   generator ran upstream either; only the indirection differs.
//! - **`RobotTrajectory`s, not `MotionPlanResponse`s, are carried between
//!   the stages.** Upstream's `solveSequenceItems` returns a
//!   `vector<MotionPlanResponse>` whose `error_code` every later stage has
//!   already established to be `SUCCESS` — `checkForOverlappingRadii` and
//!   the builder loop read only `.trajectory`. Dropping the code at the
//!   point it stops carrying information also removes upstream's
//!   `*(resp_cont.at(i).trajectory)` dereference of a pointer nothing has
//!   checked; here a `SUCCESS` response with no trajectory is rejected as
//!   [`SequenceError::Planning`] at the one place it can be observed.
//! - **`extract_blend_radii` iterates pairs, not indices.** Upstream's
//!   `for (i = 0; i < radii.size() - 1; ++i)` underflows on an empty list;
//!   see `doc/upstream-bugs.md`'s `extract-blend-radii-empty-list-underflow`.
//!   Iterating [`slice::windows`] makes the empty case a zero-iteration loop
//!   by construction rather than by the caller's guard.
//! - **`check_radii_for_overlap` takes `&mut`.** It reads a waypoint's tip
//!   frame, and this port's forward kinematics are computed through
//!   [`cspace_core::state::RobotState::update`], which needs `&mut`. Upstream is
//!   `const` because its `RobotState` caches transforms behind `mutable`
//!   members.
//! - **`hasSolver`'s two failure shapes collapse.** Upstream's
//!   `getSolverTipFrame` distinguishes "no solver" (`NoSolverException`)
//!   from "more than one tip frame" (`MoreThanOneTipFrameException`); this
//!   crate's [`solver_tip_frame`] reports both as
//!   [`MoveItErrorCode::Failure`], the collapse it already documents. No
//!   caller here distinguishes them: `is_invalid_blend_radii` only asks
//!   whether a blend frame exists.
//! - **The logger calls are dropped.** `RCLCPP_WARN_STREAM` on an invalid
//!   blend radius pair, and `RCLCPP_DEBUG_STREAM`'s per-item progress line,
//!   have no equivalent under `D1`. The warning's information is not lost:
//!   a radius zeroed by `is_invalid_blend_radii` is visible in the result,
//!   which upstream's warning only narrated.

use std::collections::HashMap;

use cspace_collision::CollisionEnv;
use cspace_core::error::{Error, MoveItErrorCode};
use cspace_core::model::RobotModel;
use cspace_core::state::Posed;
use cspace_core::trajectory::RobotTrajectory;

use crate::pilz::limits::LimitsContainer;
use crate::pilz::plan_components_builder::PlanComponentsBuilder;
use crate::pilz::trajectory_functions::{IkContext, solver_tip_frame};
use crate::pilz::trajectory_generator::{MotionPlanRequest, MotionPlanResponse};

/// One command of a motion sequence.
///
/// Upstream `moveit_msgs::msg::MotionSequenceItem`, restricted to the two
/// fields `CommandListManager` reads. Upstream's item also carries the
/// `planner_id` the planning pipeline dispatches on; here that choice is the
/// `solve_one` closure's, so there is no field for it.
#[derive(Debug, Clone, PartialEq)]
pub struct SequenceItem {
    /// The motion request. Upstream `req`.
    pub request: MotionPlanRequest,
    /// The radius of the blend joining this command to the *next* one.
    /// Upstream `blend_radius`. Must be non-negative, and zero on the last
    /// item.
    pub blend_radius: f64,
}

/// Why a motion sequence could not be planned.
///
/// One variant per upstream exception class. Upstream gives four of these
/// the same `INVALID_MOTION_PLAN` code, so a caller that only reads the code
/// cannot say which rule the list broke; [`SequenceError::error_code`]
/// reproduces that mapping for the wire, while the variant itself keeps the
/// distinction the code throws away.
#[derive(Debug, thiserror::Error)]
#[non_exhaustive]
pub enum SequenceError {
    /// Upstream `NegativeBlendRadiusException`.
    #[error("blend radius of command [{index}] is negative: {radius}")]
    NegativeBlendRadius {
        /// Position of the offending command in the list.
        index: usize,
        /// The radius that was given.
        radius: f64,
    },

    /// Upstream `LastBlendRadiusNotZeroException`.
    #[error("the last blending radius must be zero, got {radius}")]
    LastBlendRadiusNotZero {
        /// The last command's radius.
        radius: f64,
    },

    /// Upstream `StartStateSetException`.
    #[error(
        "only the first request of group {group_name:?} may carry a start state, but command [{index}] also does"
    )]
    StartStateSet {
        /// Position of the offending command in the list.
        index: usize,
        /// The group whose rule was broken.
        group_name: String,
    },

    /// Upstream `OverlappingBlendRadiiException`.
    #[error("overlapping blend radii between command [{first}] and [{second}]")]
    OverlappingBlendRadii {
        /// Position of the first of the two commands.
        first: usize,
        /// Position of the second, always `first + 1`.
        second: usize,
    },

    /// Upstream `PlanningPipelineException`, which carries the failing
    /// response's own code rather than the class's default `FAILURE`.
    #[error("could not solve request [{index}]: {code}")]
    Planning {
        /// Position of the command that could not be planned.
        index: usize,
        /// The code the per-item planner reported, or
        /// [`MoveItErrorCode::Failure`] for a `Success` response that
        /// carried no trajectory.
        code: MoveItErrorCode,
    },

    /// An error from the assembly itself — blending, trajectory append, or a
    /// group with no resolvable tip frame. Upstream lets the corresponding
    /// exceptions propagate out of `solve` unchanged.
    #[error(transparent)]
    Assembly(#[from] Error),
}

impl SequenceError {
    /// The `moveit_msgs` code upstream's matching exception carries.
    ///
    /// Four of the six map to [`MoveItErrorCode::InvalidMotionPlan`] — that
    /// collapse is upstream's, reproduced here so a caller marshalling this
    /// onto the wire produces the same value.
    pub fn error_code(&self) -> MoveItErrorCode {
        match self {
            Self::NegativeBlendRadius { .. }
            | Self::LastBlendRadiusNotZero { .. }
            | Self::OverlappingBlendRadii { .. } => MoveItErrorCode::InvalidMotionPlan,
            Self::StartStateSet { .. } => MoveItErrorCode::InvalidRobotState,
            Self::Planning { code, .. } => *code,
            Self::Assembly(Error::Code(code)) => *code,
            Self::Assembly(_) => MoveItErrorCode::Failure,
        }
    }
}

/// Plans lists of motion commands.
///
/// Upstream `CommandListManager`. See the [module docs](self) for the
/// sequence rules and the deviations.
pub struct CommandListManager<'m> {
    robot_model: &'m RobotModel,
    planner_limits: LimitsContainer,
}

impl<'m> CommandListManager<'m> {
    /// Upstream's constructor, minus the parameter plumbing.
    ///
    /// Upstream builds the limits itself from the ROS parameter server
    /// (`JointLimitsAggregator::getAggregatedLimits` plus a
    /// `cartesian_limits::ParamListener` under `robot_description_planning`)
    /// and then hands them to the blender it constructs. `D1` removes the
    /// parameter server, so the fused limits arrive as an argument, exactly
    /// as they do for every generator in this crate.
    pub fn new(robot_model: &'m RobotModel, planner_limits: LimitsContainer) -> Self {
        Self {
            robot_model,
            planner_limits,
        }
    }

    /// Upstream `CommandListManager::solve`.
    ///
    /// `solve_one` plans a single request; see the [module docs](self) for
    /// why it is the caller's. It is called once per item, in order, with
    /// the item's request carrying the start state this method sets.
    ///
    /// An empty `items` yields an empty result without calling `solve_one`,
    /// matching upstream's first statement.
    ///
    /// # Errors
    ///
    /// [`SequenceError`], one variant per upstream exception class.
    pub fn solve<E, S>(
        &self,
        ctx: &IkContext<'_, 'm, E>,
        items: &[SequenceItem],
        solve_one: S,
    ) -> Result<Vec<RobotTrajectory<'m>>, SequenceError>
    where
        E: for<'s> CollisionEnv<Posed<'s, 'm>>,
        S: FnMut(&MotionPlanRequest) -> MotionPlanResponse<'m>,
    {
        if items.is_empty() {
            return Ok(Vec::new());
        }

        Self::check_for_negative_radii(items)?;
        Self::check_last_blend_radius_zero(items)?;
        Self::check_start_states(items)?;

        let mut trajectories = self.solve_sequence_items(items, solve_one)?;

        let radii = self.extract_blend_radii(items);
        self.check_for_overlapping_radii(&mut trajectories, &radii)?;

        let mut builder = PlanComponentsBuilder::new(self.robot_model, self.planner_limits.clone());
        for (i, trajectory) in trajectories.into_iter().enumerate() {
            // The blend radius is "attached" to the second part of a blend
            // trajectory, therefore `i - 1`.
            let blend_radius = if i > 0 { radii[i - 1] } else { 0.0 };
            builder.append(ctx, trajectory, blend_radius)?;
        }
        Ok(builder.build()?)
    }

    /// Upstream `CommandListManager::solveSequenceItems`.
    fn solve_sequence_items<S>(
        &self,
        items: &[SequenceItem],
        mut solve_one: S,
    ) -> Result<Vec<RobotTrajectory<'m>>, SequenceError>
    where
        S: FnMut(&MotionPlanRequest) -> MotionPlanResponse<'m>,
    {
        let mut trajectories: Vec<RobotTrajectory<'m>> = Vec::with_capacity(items.len());
        for (index, item) in items.iter().enumerate() {
            let mut request = item.request.clone();
            self.set_start_state(&trajectories, &mut request)?;

            let response = solve_one(&request);
            match (response.error_code, response.trajectory) {
                (MoveItErrorCode::Success, Some(trajectory)) => trajectories.push(trajectory),
                // A `Success` with no trajectory is upstream's unchecked
                // `*(resp_cont.at(i).trajectory)` dereference; see the module
                // docs.
                (MoveItErrorCode::Success, None) => {
                    return Err(SequenceError::Planning {
                        index,
                        code: MoveItErrorCode::Failure,
                    });
                }
                (code, _) => return Err(SequenceError::Planning { index, code }),
            }
        }
        Ok(trajectories)
    }

    /// Upstream `CommandListManager::setStartState`, with
    /// `getPreviousEndState` inlined — it has this one caller and returns a
    /// borrow of `previous`, which is exactly the borrow this function needs.
    ///
    /// Upstream converts the found state with `robotStateToRobotStateMsg(…,
    /// false)` and sets `is_diff = true`; here the request's start state is a
    /// map of the variables to override, which is what `is_diff` meant.
    fn set_start_state(
        &self,
        previous: &[RobotTrajectory<'m>],
        request: &mut MotionPlanRequest,
    ) -> Result<(), SequenceError> {
        let Some(trajectory) = previous
            .iter()
            .rev()
            .find(|traj| traj.group_name() == request.group_name)
        else {
            return Ok(());
        };
        let state = trajectory.last_way_point().map_err(SequenceError::from)?;

        let names = self.robot_model.variable_names();
        let mut position = HashMap::with_capacity(names.len());
        for name in names {
            position.insert(name.clone(), state.variable_position(name)?);
        }
        request.start_state.position = position;

        // `robotStateToJointStateMsg` writes a velocity array only when the
        // state has one; an empty map is this port's empty array.
        request.start_state.velocity = if state.has_velocities() {
            let mut velocity = HashMap::with_capacity(names.len());
            for name in names {
                velocity.insert(name.clone(), state.variable_velocity(name)?);
            }
            velocity
        } else {
            HashMap::new()
        };
        Ok(())
    }

    /// Upstream `CommandListManager::extractBlendRadii`.
    ///
    /// A radius that [`Self::is_invalid_blend_radii`] rejects is zeroed
    /// rather than reported: upstream warns and continues, so an
    /// unblendable pair concatenates instead of failing the whole sequence.
    fn extract_blend_radii(&self, items: &[SequenceItem]) -> Vec<f64> {
        let mut radii = vec![0.0; items.len()];
        for (i, pair) in items.windows(2).enumerate() {
            if self.is_invalid_blend_radii(&pair[0], &pair[1]) {
                continue;
            }
            radii[i] = pair[0].blend_radius;
        }
        radii
    }

    /// Upstream `CommandListManager::isInvalidBlendRadii`.
    ///
    /// The first rule ("a zero blend radius is always valid") decides nothing
    /// this port can observe: [`Self::extract_blend_radii`] writes
    /// `first.blend_radius` on the valid path and leaves the slot at its
    /// initial `0.0` on the invalid one, which for a zero radius is the same
    /// value. Upstream's rule *was* observable — it suppressed a
    /// `RCLCPP_WARN_STREAM` that a zero-radius pair across groups would
    /// otherwise emit — and this port drops the logging. It is kept because
    /// it still skips the solver lookup the other two rules need, and
    /// removing it would leave `is_invalid_blend_radii` answering "invalid"
    /// for a radius that is by definition fine.
    fn is_invalid_blend_radii(&self, first: &SequenceItem, second: &SequenceItem) -> bool {
        // Zero blend radius is always valid.
        if first.blend_radius == 0.0 {
            return false;
        }
        // No blending between different groups.
        if first.request.group_name != second.request.group_name {
            return true;
        }
        // No blending for groups without solver. Upstream `hasSolver`; here
        // the same question is "does this group have a blend frame".
        solver_tip_frame(self.robot_model, &first.request.group_name).is_err()
    }

    /// Upstream `CommandListManager::checkForOverlappingRadii`.
    ///
    /// Every consecutive pair *except the last* is checked: the final pair's
    /// second radius is the last one, which
    /// [`Self::check_last_blend_radius_zero`] has already established is
    /// zero.
    ///
    /// Upstream's two early returns (`empty()`, then `size() < 3`) exist only
    /// so that `size() - 2` cannot underflow; they carry no rule of their
    /// own, since a list of one or two trajectories has no pair left after
    /// the last one is dropped. Saturating the subtraction says the same
    /// thing without a guard whose result no output can show.
    fn check_for_overlapping_radii(
        &self,
        trajectories: &mut [RobotTrajectory<'m>],
        radii: &[f64],
    ) -> Result<(), SequenceError> {
        for i in 0..trajectories.len().saturating_sub(2) {
            let (head, tail) = trajectories.split_at_mut(i + 1);
            if self.check_radii_for_overlap(&mut head[i], radii[i], &mut tail[0], radii[i + 1])? {
                return Err(SequenceError::OverlappingBlendRadii {
                    first: i,
                    second: i + 1,
                });
            }
        }
        Ok(())
    }

    /// Upstream `CommandListManager::checkRadiiForOverlap`.
    ///
    /// Two blends overlap when the distance between the two trajectories'
    /// *end* tip poses is no greater than the sum of their radii — the
    /// second trajectory's blend would then start before the first one's
    /// ended.
    fn check_radii_for_overlap(
        &self,
        traj_a: &mut RobotTrajectory<'m>,
        radii_a: f64,
        traj_b: &mut RobotTrajectory<'m>,
        radii_b: f64,
    ) -> Result<bool, SequenceError> {
        // No blending between trajectories from different groups.
        if traj_a.group_name() != traj_b.group_name() {
            return Ok(false);
        }
        let sum_radii = radii_a + radii_b;
        if sum_radii == 0.0 {
            return Ok(false);
        }

        let blend_frame = solver_tip_frame(self.robot_model, traj_a.group_name())?;
        let end_of = |traj: &mut RobotTrajectory<'m>| -> Result<_, SequenceError> {
            Ok(traj
                .last_way_point_mut()?
                .update()
                .frame_transform(&blend_frame)?
                .translation
                .vector)
        };
        let distance_endpoints = (end_of(traj_a)? - end_of(traj_b)?).norm();
        Ok(distance_endpoints <= sum_radii)
    }

    /// Upstream `CommandListManager::checkForNegativeRadii`.
    fn check_for_negative_radii(items: &[SequenceItem]) -> Result<(), SequenceError> {
        match items.iter().position(|item| item.blend_radius < 0.0) {
            Some(index) => Err(SequenceError::NegativeBlendRadius {
                index,
                radius: items[index].blend_radius,
            }),
            None => Ok(()),
        }
    }

    /// Upstream `CommandListManager::checkLastBlendRadiusZero`.
    fn check_last_blend_radius_zero(items: &[SequenceItem]) -> Result<(), SequenceError> {
        // `solve` has already returned on an empty list; upstream's
        // `items.back()` has the same precondition and does not state it.
        let radius = items[items.len() - 1].blend_radius;
        if radius == 0.0 {
            Ok(())
        } else {
            Err(SequenceError::LastBlendRadiusNotZero { radius })
        }
    }

    /// Upstream `CommandListManager::checkStartStates` with
    /// `checkStartStatesOfGroup` and `getGroupNames` folded in.
    ///
    /// Upstream collects the distinct group names, then re-walks the whole
    /// list once per group skipping the items of other groups. One walk
    /// remembering which groups have been seen answers the same question:
    /// an item may carry a start state exactly when it is the first of its
    /// group.
    fn check_start_states(items: &[SequenceItem]) -> Result<(), SequenceError> {
        if items.len() <= 1 {
            return Ok(());
        }
        let mut seen: Vec<&str> = Vec::new();
        for (index, item) in items.iter().enumerate() {
            let group_name = item.request.group_name.as_str();
            if !seen.contains(&group_name) {
                seen.push(group_name);
                continue;
            }
            if !(item.request.start_state.position.is_empty()
                && item.request.start_state.velocity.is_empty())
            {
                return Err(SequenceError::StartStateSet {
                    index,
                    group_name: group_name.to_string(),
                });
            }
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use std::cell::RefCell;
    use std::fs;
    use std::sync::Arc;

    use cspace_collision::{LinkPaddingScale, ParryCollisionEnv, World};
    use cspace_core::geometry::{UnitQuaternion, Vector3};
    use cspace_core::model::MeshSearchPaths;
    use cspace_core::srdf::SrdfModel;
    use cspace_core::state::RobotState;
    use cspace_planning::scene::PlanningScene;

    use super::*;
    use crate::pilz::limits::{CartesianLimits, JointLimit, JointLimitsContainer};
    use crate::pilz::trajectory_generator::{
        Goal, PilzGenerator, StartState, TrajectoryGenerator as PilzBase,
    };
    use crate::pilz::trajectory_generator_lin::TrajectoryGeneratorLin;

    fn load_panda() -> (RobotModel, SrdfModel) {
        let root = concat!(env!("CARGO_MANIFEST_DIR"), "/../../fixtures");
        let urdf_xml = fs::read_to_string(format!("{root}/panda.urdf")).unwrap();
        let urdf = urdf_rs::read_from_string(&urdf_xml).expect("fixture URDF must parse");
        let srdf = SrdfModel::parse_file(format!("{root}/panda.srdf")).unwrap();
        let meshes_root = concat!(env!("CARGO_MANIFEST_DIR"), "/../../fixtures/meshes");
        let mesh_paths = MeshSearchPaths::new([(
            "moveit_resources_panda_description",
            format!("{meshes_root}/panda_description"),
        )]);
        let model = RobotModel::from_urdf_and_srdf(&urdf, &urdf_xml, &srdf, &mesh_paths)
            .expect("fixture model must build");
        (model, srdf)
    }

    fn ready_positions() -> HashMap<String, f64> {
        [
            ("panda_joint1", 0.0),
            ("panda_joint2", -0.785),
            ("panda_joint3", 0.0),
            ("panda_joint4", -2.356),
            ("panda_joint5", 0.0),
            ("panda_joint6", 1.571),
            ("panda_joint7", 0.785),
        ]
        .into_iter()
        .map(|(k, v)| (k.to_string(), v))
        .collect()
    }

    fn limits() -> LimitsContainer {
        let mut joint_limits = JointLimitsContainer::default();
        for joint in [
            "panda_joint1",
            "panda_joint2",
            "panda_joint3",
            "panda_joint4",
            "panda_joint5",
            "panda_joint6",
            "panda_joint7",
        ] {
            joint_limits.add_limit(
                joint,
                JointLimit {
                    has_position_limits: true,
                    min_position: -2.9,
                    max_position: 2.9,
                    has_velocity_limits: true,
                    max_velocity: 10.0,
                    has_acceleration_limits: true,
                    max_acceleration: 100.0,
                    has_deceleration_limits: true,
                    max_deceleration: -100.0,
                    ..Default::default()
                },
            );
        }
        let mut out = LimitsContainer::new();
        out.set_joint_limits(joint_limits);
        out.set_cartesian_limits(CartesianLimits {
            max_trans_vel: 1.0,
            max_trans_acc: 2.25,
            max_trans_dec: -5.0,
            max_rot_vel: 1.57,
        });
        out
    }

    /// The `"ready"` pose orientation every LIN goal below keeps.
    fn ready_orientation() -> UnitQuaternion {
        UnitQuaternion::from_quaternion(nalgebra::Quaternion::new(
            3.2004117663522442e-12,
            0.9239556994689483,
            -0.38249949727920757,
            1.324932583900579e-12,
        ))
    }

    /// A request that never reaches a planner — every test that only exercises
    /// the four validity rules uses it, so the rule under test is the only
    /// thing that can decide the outcome.
    fn unplanned_request(group_name: &str) -> MotionPlanRequest {
        MotionPlanRequest {
            group_name: group_name.to_string(),
            start_state: StartState::default(),
            goal: Goal::Joint(HashMap::new()),
            max_velocity_scaling_factor: 0.1,
            max_acceleration_scaling_factor: 0.1,
            path_constraints: None,
        }
    }

    fn item(group_name: &str, blend_radius: f64) -> SequenceItem {
        SequenceItem {
            request: unplanned_request(group_name),
            blend_radius,
        }
    }

    fn item_with_start_state(group_name: &str, blend_radius: f64) -> SequenceItem {
        let mut request = unplanned_request(group_name);
        request.start_state.position = ready_positions();
        SequenceItem {
            request,
            blend_radius,
        }
    }

    fn cartesian_item(goal_pos: [f64; 3], blend_radius: f64) -> SequenceItem {
        let mut request = unplanned_request("panda_arm");
        request.goal = Goal::Cartesian {
            link_name: "panda_link8".to_string(),
            frame: None,
            position: Vector3::new(goal_pos[0], goal_pos[1], goal_pos[2]),
            orientation: ready_orientation(),
            target_point_offset: Vector3::new(0.0, 0.0, 0.0),
        };
        SequenceItem {
            request,
            blend_radius,
        }
    }

    /// A `panda_joint1` sweep in `group`, used where `solve` needs a
    /// trajectory but not a planned one.
    fn sweep<'m>(
        model: &'m RobotModel,
        group: &str,
        start_offset: f64,
        end_offset: f64,
        steps: usize,
    ) -> RobotTrajectory<'m> {
        let base = ready_positions();
        let mut traj = RobotTrajectory::for_group_name(model, group).unwrap();
        for i in 0..=steps {
            let mut state = RobotState::new(model);
            state.set_to_default_values();
            for (name, &value) in &base {
                state.set_variable_position(name, value).unwrap();
            }
            let angle = start_offset + (end_offset - start_offset) * (i as f64) / (steps as f64);
            state.set_variable_position("panda_joint1", angle).unwrap();
            traj.add_suffix_way_point(state, if i == 0 { 0.0 } else { 0.1 })
                .unwrap();
        }
        traj
    }

    struct Fixture<'m> {
        scene: Arc<PlanningScene<'m>>,
        env: ParryCollisionEnv,
    }

    impl<'m> Fixture<'m> {
        fn new(model: &'m RobotModel, srdf: &SrdfModel) -> Self {
            Self {
                scene: Arc::new(PlanningScene::new(model, srdf)),
                env: ParryCollisionEnv::new(World::new(), LinkPaddingScale::default()),
            }
        }

        fn ctx(&self) -> IkContext<'_, 'm, ParryCollisionEnv> {
            IkContext {
                scene: &self.scene,
                env: &self.env,
                check_self_collision: true,
            }
        }
    }

    /// A `solve_one` that plans `panda_arm` LIN motions, the shape a caller
    /// binding one of this crate's generators would write.
    fn lin_solver<'m, 'a>(
        model: &'m RobotModel,
        planner_limits: &LimitsContainer,
        ctx: &'a IkContext<'a, 'm, ParryCollisionEnv>,
    ) -> impl FnMut(&MotionPlanRequest) -> MotionPlanResponse<'m> + use<'m, 'a> {
        let generator =
            TrajectoryGeneratorLin::new(PilzBase::new(model, planner_limits.clone()), "panda_arm");
        move |req| generator.generate(ctx, req, 0.1)
    }

    // -- check_for_negative_radii ------------------------------------------

    #[test]
    fn a_negative_radius_is_rejected_and_names_its_command() {
        let items = [
            item("panda_arm", 0.1),
            item("panda_arm", -1e-12),
            item("panda_arm", 0.0),
        ];
        match CommandListManager::check_for_negative_radii(&items) {
            Err(SequenceError::NegativeBlendRadius { index, radius }) => {
                assert_eq!(index, 1);
                assert_eq!(radius, -1e-12);
            }
            other => panic!("expected NegativeBlendRadius, got {other:?}"),
        }
    }

    #[test]
    fn a_zero_radius_is_not_negative() {
        // The boundary the guard is `< 0.0` rather than `<= 0.0` for: zero is
        // the *required* value of the last radius, so rejecting it would make
        // every valid list invalid.
        let items = [item("panda_arm", 0.0), item("panda_arm", 0.0)];
        assert!(CommandListManager::check_for_negative_radii(&items).is_ok());
    }

    // -- check_last_blend_radius_zero --------------------------------------

    #[test]
    fn a_non_zero_last_radius_is_rejected() {
        let items = [item("panda_arm", 0.0), item("panda_arm", 0.1)];
        match CommandListManager::check_last_blend_radius_zero(&items) {
            Err(SequenceError::LastBlendRadiusNotZero { radius }) => assert_eq!(radius, 0.1),
            other => panic!("expected LastBlendRadiusNotZero, got {other:?}"),
        }
    }

    #[test]
    fn only_the_last_radius_has_to_be_zero() {
        // Same list as above with the two radii swapped -- the only
        // difference -- so a guard that looked at any item but the last would
        // reject this one too.
        let items = [item("panda_arm", 0.1), item("panda_arm", 0.0)];
        assert!(CommandListManager::check_last_blend_radius_zero(&items).is_ok());
    }

    // -- check_start_states -------------------------------------------------

    #[test]
    fn only_the_first_request_of_a_group_may_carry_a_start_state() {
        let items = [
            item_with_start_state("panda_arm", 0.0),
            item_with_start_state("panda_arm", 0.0),
        ];
        match CommandListManager::check_start_states(&items) {
            Err(SequenceError::StartStateSet { index, group_name }) => {
                assert_eq!(index, 1);
                assert_eq!(group_name, "panda_arm");
            }
            other => panic!("expected StartStateSet, got {other:?}"),
        }
    }

    #[test]
    fn the_first_request_of_each_group_may_carry_a_start_state() {
        // Same two start states as above; only the second item's group
        // differs, so this is the pair that shows the rule is per-group and
        // not per-list.
        let items = [
            item_with_start_state("panda_arm", 0.0),
            item_with_start_state("hand", 0.0),
        ];
        assert!(CommandListManager::check_start_states(&items).is_ok());
    }

    #[test]
    fn a_group_returning_after_another_still_counts_as_seen() {
        // panda_arm, hand, panda_arm: the third item is not the first of its
        // group even though the item before it is a different group.
        let items = [
            item_with_start_state("panda_arm", 0.0),
            item_with_start_state("hand", 0.0),
            item_with_start_state("panda_arm", 0.0),
        ];
        match CommandListManager::check_start_states(&items) {
            Err(SequenceError::StartStateSet { index, .. }) => assert_eq!(index, 2),
            other => panic!("expected StartStateSet at [2], got {other:?}"),
        }
    }

    #[test]
    fn a_single_item_list_skips_the_start_state_rule() {
        let items = [item_with_start_state("panda_arm", 0.0)];
        assert!(CommandListManager::check_start_states(&items).is_ok());
    }

    // -- extract_blend_radii / is_invalid_blend_radii ----------------------

    #[test]
    fn a_valid_radius_survives_extraction_and_the_last_is_always_zero() {
        let (model, _) = load_panda();
        let manager = CommandListManager::new(&model, limits());
        let items = [item("panda_arm", 0.05), item("panda_arm", 0.0)];
        assert_eq!(manager.extract_blend_radii(&items), vec![0.05, 0.0]);
    }

    #[test]
    fn a_radius_spanning_a_group_change_is_zeroed() {
        // Same 0.05 as the test above; only the second item's group differs.
        let (model, _) = load_panda();
        let manager = CommandListManager::new(&model, limits());
        let items = [item("panda_arm", 0.05), item("hand", 0.0)];
        assert_eq!(manager.extract_blend_radii(&items), vec![0.0, 0.0]);
    }

    #[test]
    fn a_radius_in_a_group_without_a_solver_is_zeroed() {
        // Both items are `hand`, so the group-change rule cannot fire and the
        // no-solver rule is the only one left that can zero this radius.
        let (model, _) = load_panda();
        assert!(
            solver_tip_frame(&model, "hand").is_err(),
            "this test needs a group with no solver; if `hand` gains one, pick \
             another group"
        );
        let manager = CommandListManager::new(&model, limits());
        let items = [item("hand", 0.05), item("hand", 0.0)];
        assert_eq!(manager.extract_blend_radii(&items), vec![0.0, 0.0]);
    }

    // -- check_for_overlapping_radii ---------------------------------------

    #[test]
    fn the_last_pair_of_trajectories_is_never_checked() {
        let (model, _) = load_panda();
        let manager = CommandListManager::new(&model, limits());
        let mut trajectories = vec![
            sweep(&model, "panda_arm", 0.0, 0.2, 2),
            sweep(&model, "panda_arm", 0.2, 0.4, 2),
        ];
        // Coincident-enough endpoints and radii large enough that the check
        // would fire on this pair if it were checked; it is the list's last
        // pair, so it is not.
        assert!(
            manager
                .check_for_overlapping_radii(&mut trajectories, &[10.0, 10.0])
                .is_ok()
        );
    }

    #[test]
    fn radii_summing_past_the_endpoint_distance_overlap() {
        let (model, _) = load_panda();
        let manager = CommandListManager::new(&model, limits());
        let mut trajectories = vec![
            sweep(&model, "panda_arm", 0.0, 0.05, 2),
            sweep(&model, "panda_arm", 0.05, 0.10, 2),
            sweep(&model, "panda_arm", 0.10, 0.15, 2),
        ];
        match manager.check_for_overlapping_radii(&mut trajectories, &[10.0, 10.0, 0.0]) {
            Err(SequenceError::OverlappingBlendRadii { first, second }) => {
                assert_eq!((first, second), (0, 1));
            }
            other => panic!("expected OverlappingBlendRadii, got {other:?}"),
        }
    }

    #[test]
    fn radii_below_the_endpoint_distance_do_not_overlap() {
        // Identical to the test above except for the two radii, so the
        // endpoint distance is held fixed and only the sum moves.
        let (model, _) = load_panda();
        let manager = CommandListManager::new(&model, limits());
        let mut trajectories = vec![
            sweep(&model, "panda_arm", 0.0, 0.05, 2),
            sweep(&model, "panda_arm", 0.05, 0.10, 2),
            sweep(&model, "panda_arm", 0.10, 0.15, 2),
        ];
        assert!(
            manager
                .check_for_overlapping_radii(&mut trajectories, &[1e-9, 1e-9, 0.0])
                .is_ok()
        );
    }

    #[test]
    fn two_zero_radii_never_overlap_however_close_the_endpoints() {
        let (model, _) = load_panda();
        let manager = CommandListManager::new(&model, limits());
        // Three trajectories that all end at the same pose: the endpoint
        // distance is exactly zero, so only the `sum_radii == 0` guard can
        // make this pass.
        let mut trajectories = vec![
            sweep(&model, "panda_arm", 0.0, 0.1, 2),
            sweep(&model, "panda_arm", 0.0, 0.1, 2),
            sweep(&model, "panda_arm", 0.0, 0.1, 2),
        ];
        assert!(
            manager
                .check_for_overlapping_radii(&mut trajectories, &[0.0, 0.0, 0.0])
                .is_ok()
        );
    }

    #[test]
    fn trajectories_of_different_groups_never_overlap() {
        let (model, _) = load_panda();
        let manager = CommandListManager::new(&model, limits());
        // Same coincident endpoints as the test above, but non-zero radii, so
        // the group check is the only guard left that can pass this.
        let mut trajectories = vec![
            sweep(&model, "panda_arm", 0.0, 0.1, 2),
            sweep(&model, "hand", 0.0, 0.1, 2),
            sweep(&model, "panda_arm", 0.0, 0.1, 2),
        ];
        assert!(
            manager
                .check_for_overlapping_radii(&mut trajectories, &[10.0, 10.0, 0.0])
                .is_ok()
        );
    }

    // -- set_start_state ---------------------------------------------------

    #[test]
    fn the_next_request_of_a_group_starts_where_the_previous_one_ended() {
        let (model, _) = load_panda();
        let manager = CommandListManager::new(&model, limits());
        let previous = vec![sweep(&model, "panda_arm", 0.0, 0.25, 4)];
        let mut request = unplanned_request("panda_arm");
        manager.set_start_state(&previous, &mut request).unwrap();
        assert_eq!(
            request.start_state.position.get("panda_joint1"),
            Some(&0.25)
        );
        assert_eq!(
            request.start_state.position.get("panda_joint4"),
            Some(&-2.356),
            "every model variable is carried, not only the swept one"
        );
    }

    #[test]
    fn the_first_request_of_a_group_keeps_its_own_start_state() {
        // Same `previous` as above; only the request's group differs, so the
        // per-group lookup is the only thing that can leave it untouched.
        let (model, _) = load_panda();
        let manager = CommandListManager::new(&model, limits());
        let previous = vec![sweep(&model, "panda_arm", 0.0, 0.25, 4)];
        let mut request = unplanned_request("hand");
        request.start_state.position = ready_positions();
        manager.set_start_state(&previous, &mut request).unwrap();
        assert_eq!(request.start_state.position, ready_positions());
    }

    #[test]
    fn the_most_recent_trajectory_of_the_group_wins() {
        let (model, _) = load_panda();
        let manager = CommandListManager::new(&model, limits());
        let previous = vec![
            sweep(&model, "panda_arm", 0.0, 0.25, 4),
            sweep(&model, "hand", 0.0, 0.5, 4),
            sweep(&model, "panda_arm", 0.25, 0.40, 4),
        ];
        let mut request = unplanned_request("panda_arm");
        manager.set_start_state(&previous, &mut request).unwrap();
        assert_eq!(
            request.start_state.position.get("panda_joint1"),
            Some(&0.40)
        );
    }

    // -- solve --------------------------------------------------------------

    #[test]
    fn an_empty_list_yields_an_empty_result_without_planning() {
        let (model, srdf) = load_panda();
        let fixture = Fixture::new(&model, &srdf);
        let manager = CommandListManager::new(&model, limits());
        let calls = RefCell::new(0usize);
        let built = manager
            .solve(&fixture.ctx(), &[], |_| {
                *calls.borrow_mut() += 1;
                MotionPlanResponse {
                    error_code: MoveItErrorCode::Failure,
                    trajectory: None,
                }
            })
            .unwrap();
        assert!(built.is_empty());
        assert_eq!(*calls.borrow(), 0);
    }

    #[test]
    fn a_failing_item_stops_the_sequence_and_names_its_index_and_code() {
        let (model, srdf) = load_panda();
        let fixture = Fixture::new(&model, &srdf);
        let manager = CommandListManager::new(&model, limits());
        let calls = RefCell::new(0usize);
        let result = manager.solve(
            &fixture.ctx(),
            &[item("panda_arm", 0.0), item("panda_arm", 0.0)],
            |_| {
                let n = {
                    let mut c = calls.borrow_mut();
                    *c += 1;
                    *c
                };
                if n == 1 {
                    MotionPlanResponse {
                        error_code: MoveItErrorCode::Success,
                        trajectory: Some(sweep(&model, "panda_arm", 0.0, 0.1, 2)),
                    }
                } else {
                    MotionPlanResponse {
                        error_code: MoveItErrorCode::NoIkSolution,
                        trajectory: None,
                    }
                }
            },
        );
        match result {
            Err(SequenceError::Planning { index, code }) => {
                assert_eq!(index, 1);
                assert_eq!(code, MoveItErrorCode::NoIkSolution);
            }
            other => panic!("expected Planning, got {other:?}"),
        }
        assert_eq!(*calls.borrow(), 2, "planning must stop at the failure");
    }

    #[test]
    fn a_success_carrying_no_trajectory_is_a_planning_failure() {
        let (model, srdf) = load_panda();
        let fixture = Fixture::new(&model, &srdf);
        let manager = CommandListManager::new(&model, limits());
        let result = manager.solve(&fixture.ctx(), &[item("panda_arm", 0.0)], |_| {
            MotionPlanResponse {
                error_code: MoveItErrorCode::Success,
                trajectory: None,
            }
        });
        match result {
            Err(SequenceError::Planning { index, code }) => {
                assert_eq!(index, 0);
                assert_eq!(code, MoveItErrorCode::Failure);
            }
            other => panic!("expected Planning, got {other:?}"),
        }
    }

    #[test]
    fn a_two_command_sequence_is_planned_chained_and_blended() {
        let (model, srdf) = load_panda();
        let fixture = Fixture::new(&model, &srdf);
        let ctx = fixture.ctx();
        let planner_limits = limits();
        let manager = CommandListManager::new(&model, planner_limits.clone());

        // The right-angle corner `trajectory_blender_transition_window`'s own
        // geometry tests pin.
        let corner = [
            0.40701957005161055,
            -5.221329615610066e-12,
            0.5902695582766445,
        ];
        let second = [corner[0], 0.1, corner[2]];

        let mut first_item = cartesian_item(corner, 0.05);
        first_item.request.start_state.position = ready_positions();
        let items = [first_item, cartesian_item(second, 0.0)];

        let blended = manager
            .solve(&ctx, &items, lin_solver(&model, &planner_limits, &ctx))
            .expect("the sequence must plan");

        let mut zero_radius = items.clone();
        zero_radius[0].blend_radius = 0.0;
        let concatenated = manager
            .solve(
                &ctx,
                &zero_radius,
                lin_solver(&model, &planner_limits, &ctx),
            )
            .expect("the zero-radius control must plan");

        assert_eq!(blended.len(), 1, "one group means one output trajectory");
        assert_eq!(concatenated.len(), 1);
        assert_ne!(
            blended[0].way_point_count(),
            concatenated[0].way_point_count(),
            "a positive blend radius must reach the blender"
        );
        assert!(
            blended[0].duration() < concatenated[0].duration(),
            "rounding the corner must take less time than stopping at it: {} vs {}",
            blended[0].duration(),
            concatenated[0].duration()
        );
    }

    #[test]
    fn the_second_command_is_planned_from_the_first_ones_end_state() {
        // The second item carries no start state; if `set_start_state` did
        // not chain it, LIN would plan from the model's default pose and the
        // two segments would not meet.
        let (model, srdf) = load_panda();
        let fixture = Fixture::new(&model, &srdf);
        let ctx = fixture.ctx();
        let planner_limits = limits();
        let manager = CommandListManager::new(&model, planner_limits.clone());

        let corner = [
            0.40701957005161055,
            -5.221329615610066e-12,
            0.5902695582766445,
        ];
        let mut first_item = cartesian_item(corner, 0.0);
        first_item.request.start_state.position = ready_positions();
        let items = [first_item, cartesian_item([corner[0], 0.1, corner[2]], 0.0)];

        let built = manager
            .solve(&ctx, &items, lin_solver(&model, &planner_limits, &ctx))
            .expect("the sequence must plan");
        assert_eq!(built.len(), 1);

        let group = model.joint_model_group("panda_arm").unwrap();
        let mut worst = 0.0f64;
        for i in 1..built[0].way_point_count() {
            for name in group.active_joint_names() {
                let a = built[0]
                    .way_point(i - 1)
                    .unwrap()
                    .variable_position(name)
                    .unwrap();
                let b = built[0]
                    .way_point(i)
                    .unwrap()
                    .variable_position(name)
                    .unwrap();
                worst = worst.max((b - a).abs());
            }
        }
        // Measured over this fixture: `3.12e-2 rad`, the LIN sampling step at
        // 0.1 s. Without the chaining the second segment would restart from
        // the model's default pose, whose `panda_joint4` alone is 2.356 rad
        // from the corner -- two orders above this bound, so the threshold
        // sits far from both values rather than being tuned to either.
        assert!(
            worst < 0.1,
            "the junction must be continuous; worst consecutive joint delta was {worst:e}"
        );
    }

    // -- error_code ---------------------------------------------------------

    #[test]
    fn every_variant_reports_the_code_its_upstream_exception_carries() {
        assert_eq!(
            SequenceError::NegativeBlendRadius {
                index: 0,
                radius: -1.0
            }
            .error_code(),
            MoveItErrorCode::InvalidMotionPlan
        );
        assert_eq!(
            SequenceError::LastBlendRadiusNotZero { radius: 1.0 }.error_code(),
            MoveItErrorCode::InvalidMotionPlan
        );
        assert_eq!(
            SequenceError::OverlappingBlendRadii {
                first: 0,
                second: 1
            }
            .error_code(),
            MoveItErrorCode::InvalidMotionPlan
        );
        assert_eq!(
            SequenceError::StartStateSet {
                index: 1,
                group_name: "panda_arm".to_string()
            }
            .error_code(),
            MoveItErrorCode::InvalidRobotState,
            "the one variant upstream does NOT give INVALID_MOTION_PLAN"
        );
        assert_eq!(
            SequenceError::Planning {
                index: 0,
                code: MoveItErrorCode::NoIkSolution
            }
            .error_code(),
            MoveItErrorCode::NoIkSolution,
            "upstream passes the response's own code, not the class default"
        );
        assert_eq!(
            SequenceError::Assembly(Error::Code(MoveItErrorCode::InvalidGroupName)).error_code(),
            MoveItErrorCode::InvalidGroupName
        );
        assert_eq!(
            SequenceError::Assembly(Error::construct("no")).error_code(),
            MoveItErrorCode::Failure
        );
    }
}
