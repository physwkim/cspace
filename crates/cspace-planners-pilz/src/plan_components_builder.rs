// Copyright (c) 2018, Pilz GmbH & Co. KG
// Copyright (c) 2019, Pilz GmbH & Co. KG
// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// Ported from moveit2 @ e017c91ee12984393a28ba246075c65f69cde3bf:
//   moveit_planners/pilz_industrial_motion_planner/include/pilz_industrial_motion_planner/plan_components_builder.hpp
//   moveit_planners/pilz_industrial_motion_planner/src/plan_components_builder.cpp

//! Assembly of a sequence of planned trajectories into the list of
//! trajectories a controller executes ([`PlanComponentsBuilder`]).
//!
//! Upstream `PlanComponentsBuilder`. One trajectory at a time is
//! [`append`](PlanComponentsBuilder::append)ed with the blend radius that
//! joins it to its predecessor, and [`build`](PlanComponentsBuilder::build)
//! returns the result. Three cases, exactly upstream's:
//!
//! - a different group than the previous trajectory starts a new element of
//!   the output list — a controller cannot execute two groups from one
//!   trajectory;
//! - the same group with `blend_radius <= 0.0` is concatenated onto the
//!   current element;
//! - the same group with `blend_radius > 0.0` is blended into it by
//!   [`crate::trajectory_blender_transition_window::blend`].
//!
//! # Deviations from upstream
//!
//! - **The three "not configured" exceptions are unrepresentable.**
//!   Upstream's builder is default-constructed and then configured by
//!   `setModel`/`setBlender`, so `append` must check `model_` and `blend`
//!   must check `blender_`, raising `NoRobotModelSetException` /
//!   `NoBlenderSetException`. [`PlanComponentsBuilder::new`] takes the robot
//!   model and the limits the blender needs, so neither state exists to
//!   check for. `NoTipFrameFunctionSetException` and its `TipFrameFunc_t`
//!   are declared in upstream's header and thrown and called nowhere in the
//!   package — they are dead declarations, not ported.
//! - **`build` consumes the builder.** Upstream's `build() const` copies the
//!   `vector<RobotTrajectoryPtr>` but not the trajectories, then appends
//!   `traj_tail_` through the copied pointer — mutating the builder's own
//!   last element from a `const` method, so a second `build()` appends the
//!   tail a second time. Taking `self` makes the second call unrepresentable
//!   rather than merely unreached; see `doc/upstream-bugs.md`'s
//!   `plan-components-builder-const-build-mutates`.
//! - **`reset` is not ported.** It exists upstream because the builder is a
//!   long-lived member of `CommandListManager`, reset once per `solve`. With
//!   a consuming `build`, the way to start a new sequence is a new builder,
//!   which is what `reset` was emulating.
//! - **Trajectories are owned, not shared.** Upstream passes and stores
//!   `RobotTrajectoryPtr`, so `build`'s caller and the builder alias the same
//!   trajectories. [`RobotTrajectory`] is moved in and out here, the same
//!   deviation [`crate::trajectory_blender_transition_window::TrajectoryBlendRequest`]
//!   already documents for its own two trajectories.
//! - **`assert(other->getGroupName() == traj_tail_->getGroupName())` is
//!   dropped.** It guards `blend`, which is private and called from exactly
//!   one place — the branch of `append` that has just tested that equality.
//!   Here `blend` is a private method with the same single caller, so the
//!   assertion restates its caller's own condition.

use cspace_collision::CollisionEnv;
use cspace_error::Result;
use cspace_model::RobotModel;
use cspace_state::Posed;
use cspace_trajectory::RobotTrajectory;

use crate::limits::LimitsContainer;
use crate::trajectory_blender_transition_window::{TrajectoryBlendRequest, blend};
use crate::trajectory_functions::{IkContext, is_robot_state_equal, solver_tip_frame};

/// Tolerance for comparing the joining waypoints of two trajectories.
/// Upstream `PlanComponentsBuilder::ROBOT_STATE_EQUALITY_EPSILON`.
pub const ROBOT_STATE_EQUALITY_EPSILON: f64 = 1e-4;

/// Merges and blends a sequence of planned trajectories.
///
/// Upstream `PlanComponentsBuilder`. See the [module docs](self) for the
/// three append cases and for the deviations.
pub struct PlanComponentsBuilder<'m> {
    /// Upstream `model_`, non-optional here.
    robot_model: &'m RobotModel,
    /// The limits upstream hands to the `TrajectoryBlenderTransitionWindow`
    /// it constructs in `setBlender`.
    planner_limits: LimitsContainer,
    /// Upstream `traj_tail_`: the previously appended trajectory, held back
    /// so the next `append` can decide whether to blend into it.
    traj_tail: Option<RobotTrajectory<'m>>,
    /// Upstream `traj_cont_`: the output list under construction.
    traj_cont: Vec<RobotTrajectory<'m>>,
}

impl<'m> PlanComponentsBuilder<'m> {
    /// Upstream's default construction followed by `setModel` and
    /// `setBlender`, which this port cannot separate — see the
    /// [module docs](self).
    pub fn new(robot_model: &'m RobotModel, planner_limits: LimitsContainer) -> Self {
        Self {
            robot_model,
            planner_limits,
            traj_tail: None,
            traj_cont: Vec::new(),
        }
    }

    /// Upstream `PlanComponentsBuilder::append`.
    ///
    /// `blend_radius` joins `other` to the *previous* trajectory, so the
    /// first call's radius is unused, exactly as upstream's caller
    /// (`CommandListManager::solve` passes `0.0` for `i == 0`).
    ///
    /// # Errors
    ///
    /// Whatever [`crate::trajectory_blender_transition_window::blend`]
    /// returns when a blend is attempted and fails — upstream collapses all
    /// of them into one `BlendingFailedException` carrying
    /// [`cspace_error::MoveItErrorCode::Failure`], which loses the blender's
    /// own reason; this port propagates it. Also whatever
    /// [`RobotTrajectory::append`] returns.
    pub fn append<E>(
        &mut self,
        ctx: &IkContext<'_, 'm, E>,
        other: RobotTrajectory<'m>,
        blend_radius: f64,
    ) -> Result<()>
    where
        E: for<'s> CollisionEnv<Posed<'s, 'm>>,
    {
        let Some(tail) = self.traj_tail.take() else {
            // Reserve space in the container for the new trajectory.
            self.traj_cont.push(RobotTrajectory::for_group_name(
                self.robot_model,
                other.group_name(),
            )?);
            self.traj_tail = Some(other);
            return Ok(());
        };

        // Create a new trajectory for every group change.
        if other.group_name() != tail.group_name() {
            Self::append_with_strict_time_increase(self.last_mut(), &tail)?;
            self.traj_cont.push(RobotTrajectory::for_group_name(
                self.robot_model,
                other.group_name(),
            )?);
            self.traj_tail = Some(other);
            return Ok(());
        }

        // No blending.
        if blend_radius <= 0.0 {
            Self::append_with_strict_time_increase(self.last_mut(), &tail)?;
            self.traj_tail = Some(other);
            return Ok(());
        }

        self.blend(ctx, tail, other, blend_radius)
    }

    /// Upstream `PlanComponentsBuilder::build`, consuming — see the
    /// [module docs](self).
    ///
    /// # Errors
    ///
    /// Whatever [`RobotTrajectory::append`] returns while flushing the tail.
    pub fn build(mut self) -> Result<Vec<RobotTrajectory<'m>>> {
        if let Some(tail) = self.traj_tail.take() {
            // Upstream asserts the container is non-empty here. It cannot be:
            // `traj_tail` is only ever set by `append`, which pushes a
            // container element on the same path.
            Self::append_with_strict_time_increase(self.last_mut(), &tail)?;
        }
        Ok(self.traj_cont)
    }

    /// Upstream `PlanComponentsBuilder::blend`.
    fn blend<E>(
        &mut self,
        ctx: &IkContext<'_, 'm, E>,
        tail: RobotTrajectory<'m>,
        other: RobotTrajectory<'m>,
        blend_radius: f64,
    ) -> Result<()>
    where
        E: for<'s> CollisionEnv<Posed<'s, 'm>>,
    {
        let group_name = tail.group_name().to_string();
        let link_name = solver_tip_frame(self.robot_model, &group_name)?;
        let mut request = TrajectoryBlendRequest {
            group_name,
            link_name,
            first_trajectory: tail,
            second_trajectory: other,
            blend_radius,
        };

        let response = blend(ctx, &self.planner_limits, &mut request)?;

        Self::append_with_strict_time_increase(self.last_mut(), &response.first_trajectory)?;
        Self::append_with_strict_time_increase(self.last_mut(), &response.blend_trajectory)?;

        // The blend's own second part is the first part of the next blend.
        self.traj_tail = Some(response.second_trajectory);
        Ok(())
    }

    /// The container element `append`/`blend`/`build` are writing into.
    ///
    /// Every caller reaches this only after a `traj_cont.push`, so the
    /// container is never empty here; upstream's `traj_cont_.back()` has the
    /// same precondition and states it as an `assert` in `build` alone.
    fn last_mut(&mut self) -> &mut RobotTrajectory<'m> {
        self.traj_cont
            .last_mut()
            .expect("append pushes a container element before anything writes into one")
    }

    /// Upstream `PlanComponentsBuilder::appendWithStrictTimeIncrease`.
    ///
    /// Joint-trajectory controllers require strictly increasing times, so a
    /// repeated boundary waypoint is dropped rather than appended with a
    /// zero duration.
    ///
    /// # Errors
    ///
    /// Whatever [`RobotTrajectory::append`] or
    /// [`RobotTrajectory::add_suffix_way_point`] returns.
    fn append_with_strict_time_increase(
        result: &mut RobotTrajectory<'m>,
        source: &RobotTrajectory<'m>,
    ) -> Result<()> {
        if result.is_empty() {
            result.append(source, 0.0, 0, source.way_point_count())?;
            return Ok(());
        }

        // `last_way_point`/`first_way_point` are `Err` only on an empty
        // trajectory: `result` is non-empty by the branch above, and an empty
        // `source` makes both the comparison and the loop below vacuous, so
        // it takes the same no-op path upstream's `append` of an empty range
        // does.
        let (Ok(result_last), Ok(source_first)) =
            (result.last_way_point(), source.first_way_point())
        else {
            return Ok(());
        };

        if !is_robot_state_equal(
            result_last,
            source_first,
            result.group_name(),
            ROBOT_STATE_EQUALITY_EPSILON,
        ) {
            let dt = source.way_point_duration_from_start(0);
            result.append(source, dt, 0, source.way_point_count())?;
            return Ok(());
        }

        for i in 1..source.way_point_count() {
            result.add_suffix_way_point(
                source.way_point(i)?.clone(),
                source.way_point_duration_from_previous(i),
            )?;
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use std::collections::HashMap;
    use std::fs;
    use std::sync::Arc;

    use cspace_collision::{LinkPaddingScale, ParryCollisionEnv, World};
    use cspace_geometry::{UnitQuaternion, Vector3};
    use cspace_model::{MeshSearchPaths, RobotModel};
    use cspace_scene::PlanningScene;
    use cspace_srdf::SrdfModel;
    use cspace_state::RobotState;

    use super::*;
    use crate::limits::{CartesianLimits, JointLimit, JointLimitsContainer};
    use crate::trajectory_generator::{
        Goal, MotionPlanRequest, PilzGenerator, StartState, TrajectoryGenerator,
    };
    use crate::trajectory_generator_lin::TrajectoryGeneratorLin;

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

    /// A `panda_joint1` sweep from `start_offset` to `end_offset` about the
    /// `"ready"` pose, in `group`, with `steps + 1` waypoints. Two calls
    /// sharing an offset produce trajectories whose joining waypoints are
    /// bit-identical, which is what
    /// [`PlanComponentsBuilder::append_with_strict_time_increase`]'s
    /// equal-boundary branch needs; two calls that do not share one produce
    /// the other branch's input.
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

    fn panda_joint_limits() -> JointLimitsContainer {
        let mut limits = JointLimitsContainer::default();
        for joint in [
            "panda_joint1",
            "panda_joint2",
            "panda_joint3",
            "panda_joint4",
            "panda_joint5",
            "panda_joint6",
            "panda_joint7",
        ] {
            limits.add_limit(
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
        limits
    }

    fn blend_limits() -> LimitsContainer {
        let mut limits = LimitsContainer::new();
        limits.set_joint_limits(panda_joint_limits());
        limits.set_cartesian_limits(CartesianLimits {
            max_trans_vel: 1.0,
            max_trans_acc: 2.25,
            max_trans_dec: -5.0,
            max_rot_vel: 1.57,
        });
        limits
    }

    /// One LIN segment to `goal_pos`, the same helper shape (and the same
    /// `"ready"`-pose orientation) `trajectory_blender_transition_window`'s
    /// own geometry tests use.
    fn lin_segment<'m>(
        model: &'m RobotModel,
        limits: &LimitsContainer,
        ctx: &IkContext<'_, 'm, ParryCollisionEnv>,
        start: &HashMap<String, f64>,
        goal_pos: [f64; 3],
    ) -> RobotTrajectory<'m> {
        let base = TrajectoryGenerator::new(model, limits.clone());
        let generator = TrajectoryGeneratorLin::new(base, "panda_arm");
        let req = MotionPlanRequest {
            group_name: "panda_arm".to_string(),
            start_state: StartState {
                position: start.clone(),
                velocity: HashMap::new(),
            },
            goal: Goal::Cartesian {
                link_name: "panda_link8".to_string(),
                frame: None,
                position: Vector3::new(goal_pos[0], goal_pos[1], goal_pos[2]),
                orientation: UnitQuaternion::from_quaternion(nalgebra::Quaternion::new(
                    3.2004117663522442e-12,
                    0.9239556994689483,
                    -0.38249949727920757,
                    1.324932583900579e-12,
                )),
                target_point_offset: Vector3::new(0.0, 0.0, 0.0),
            },
            max_velocity_scaling_factor: 0.1,
            max_acceleration_scaling_factor: 0.1,
            path_constraints: None,
        };
        let response = generator.generate(ctx, &req, 0.1);
        response
            .trajectory
            .unwrap_or_else(|| panic!("LIN segment must succeed, got {:?}", response.error_code))
    }

    fn joint_positions(traj: &RobotTrajectory<'_>, index: usize) -> HashMap<String, f64> {
        let group = traj.robot_model().joint_model_group("panda_arm").unwrap();
        let state = traj.way_point(index).unwrap();
        group
            .active_joint_names()
            .iter()
            .map(|name| (name.clone(), state.variable_position(name).unwrap()))
            .collect()
    }

    /// A context with no world obstacles — every trajectory below is a
    /// free-space `panda_arm` motion, the same setup
    /// `trajectory_blender_transition_window`'s geometry tests use.
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

    #[test]
    fn build_of_an_untouched_builder_is_empty() {
        let (model, _) = load_panda();
        let built = PlanComponentsBuilder::new(&model, blend_limits())
            .build()
            .unwrap();
        assert!(built.is_empty());
    }

    #[test]
    fn the_first_append_starts_one_container_element_that_build_flushes_the_tail_into() {
        let (model, srdf) = load_panda();
        let fixture = Fixture::new(&model, &srdf);
        let traj = sweep(&model, "panda_arm", 0.0, 0.2, 4);
        let expected = traj.way_point_count();

        let mut builder = PlanComponentsBuilder::new(&model, blend_limits());
        builder.append(&fixture.ctx(), traj, 0.0).unwrap();
        let built = builder.build().unwrap();

        assert_eq!(built.len(), 1);
        assert_eq!(
            built[0].way_point_count(),
            expected,
            "build must flush the tail into the element the first append reserved"
        );
    }

    #[test]
    fn a_group_change_starts_a_second_container_element() {
        let (model, srdf) = load_panda();
        let fixture = Fixture::new(&model, &srdf);
        let arm = sweep(&model, "panda_arm", 0.0, 0.2, 4);
        let hand = sweep(&model, "hand", 0.0, 0.2, 3);
        let (arm_count, hand_count) = (arm.way_point_count(), hand.way_point_count());

        let mut builder = PlanComponentsBuilder::new(&model, blend_limits());
        builder.append(&fixture.ctx(), arm, 0.0).unwrap();
        // A non-zero radius on a group change must still not blend: the group
        // check is tested before the radius one.
        builder.append(&fixture.ctx(), hand, 0.05).unwrap();
        let built = builder.build().unwrap();

        assert_eq!(built.len(), 2, "a group change must start a new element");
        assert_eq!(built[0].group_name(), "panda_arm");
        assert_eq!(built[1].group_name(), "hand");
        assert_eq!(built[0].way_point_count(), arm_count);
        assert_eq!(built[1].way_point_count(), hand_count);
    }

    #[test]
    fn a_zero_radius_concatenates_and_drops_the_repeated_boundary_waypoint() {
        let (model, srdf) = load_panda();
        let fixture = Fixture::new(&model, &srdf);
        // The two sweeps meet at offset 0.2, so the second's first waypoint
        // is the first's last waypoint exactly.
        let first = sweep(&model, "panda_arm", 0.0, 0.2, 4);
        let second = sweep(&model, "panda_arm", 0.2, 0.4, 4);
        let (n1, n2) = (first.way_point_count(), second.way_point_count());

        let mut builder = PlanComponentsBuilder::new(&model, blend_limits());
        builder.append(&fixture.ctx(), first, 0.0).unwrap();
        builder.append(&fixture.ctx(), second, 0.0).unwrap();
        let built = builder.build().unwrap();

        assert_eq!(built.len(), 1);
        assert_eq!(
            built[0].way_point_count(),
            n1 + n2 - 1,
            "the repeated boundary waypoint must be dropped, not appended with \
             a zero duration"
        );
    }

    #[test]
    fn a_distinct_boundary_waypoint_is_kept() {
        let (model, srdf) = load_panda();
        let fixture = Fixture::new(&model, &srdf);
        // Same shape as the test above, except the second sweep starts at
        // 0.3 where the first ended at 0.2 -- the only difference.
        let first = sweep(&model, "panda_arm", 0.0, 0.2, 4);
        let second = sweep(&model, "panda_arm", 0.3, 0.5, 4);
        let (n1, n2) = (first.way_point_count(), second.way_point_count());

        let mut builder = PlanComponentsBuilder::new(&model, blend_limits());
        builder.append(&fixture.ctx(), first, 0.0).unwrap();
        builder.append(&fixture.ctx(), second, 0.0).unwrap();
        let built = builder.build().unwrap();

        assert_eq!(built.len(), 1);
        assert_eq!(
            built[0].way_point_count(),
            n1 + n2,
            "a boundary waypoint that differs from the previous trajectory's \
             last must be kept"
        );
    }

    #[test]
    fn a_positive_radius_blends_rather_than_concatenating() {
        let (model, srdf) = load_panda();
        let fixture = Fixture::new(&model, &srdf);
        let ctx = fixture.ctx();
        let limits = blend_limits();

        // The right-angle corner `trajectory_blender_transition_window`'s own
        // geometry tests pin at radius 0.05.
        let corner = [
            0.40701957005161055,
            -5.221329615610066e-12,
            0.5902695582766445,
        ];
        let first = lin_segment(&model, &limits, &ctx, &ready_positions(), corner);
        let chained = joint_positions(&first, first.way_point_count() - 1);
        let second = lin_segment(&model, &limits, &ctx, &chained, [corner[0], 0.1, corner[2]]);
        let (n1, n2) = (first.way_point_count(), second.way_point_count());

        let mut blended = PlanComponentsBuilder::new(&model, limits.clone());
        blended
            .append(&ctx, first.clone(), 0.0)
            .and_then(|()| blended.append(&ctx, second.clone(), 0.05))
            .unwrap();
        let blended = blended.build().unwrap();

        let mut concatenated = PlanComponentsBuilder::new(&model, limits);
        concatenated
            .append(&ctx, first, 0.0)
            .and_then(|()| concatenated.append(&ctx, second, 0.0))
            .unwrap();
        let concatenated = concatenated.build().unwrap();

        assert_eq!(blended.len(), 1);
        assert_eq!(concatenated.len(), 1);
        assert_eq!(
            concatenated[0].way_point_count(),
            n1 + n2 - 1,
            "the zero-radius control must be the plain concatenation"
        );
        assert_ne!(
            blended[0].way_point_count(),
            concatenated[0].way_point_count(),
            "a positive blend radius must route through the blender, not the \
             concatenation branch"
        );
        assert!(
            blended[0].duration() < concatenated[0].duration(),
            "rounding the corner must take less time than stopping at it: got \
             {} blended vs {} concatenated",
            blended[0].duration(),
            concatenated[0].duration()
        );
    }
}
