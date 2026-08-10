// Copyright (c) 2012, Willow Garage, Inc.
// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// Ported from moveit2 @ e017c91ee12984393a28ba246075c65f69cde3bf:
//   moveit_ros/planning/planning_response_adapter_plugins/src/add_time_optimal_parameterization.cpp

//! `default_planning_response_adapters::AddTimeOptimalParameterization`.
//!
//! # Symbol audit: every `rclcpp`/`moveit_msgs` occurrence (cpp:37-102)
//!
//! `rclcpp` — 6 occurrences, all logging or ROS-parameter plumbing,
//! discarded (see [`crate::request_adapters::CheckStartStateBounds`]'s
//! module doc for the same "Deviation": `path_tolerance`/`resample_dt`/
//! `min_angle_change` are plain constructor arguments here, not a
//! `ParamListener`): `rclcpp/`-namespaced includes and the
//! `rclcpp::Node::SharedPtr` parameter of `initialize` (cpp:58), one
//! `RCLCPP_DEBUG` (cpp:72), one `RCLCPP_ERROR` (cpp:75), one `RCLCPP_ERROR`
//! (cpp:89), the `rclcpp::Logger logger_` field (cpp:96) and constructor
//! initializer (cpp:52).
//!
//! `moveit_msgs` — 1 occurrence, computation, ported:
//! `moveit_msgs::msg::MoveItErrorCodes::{SUCCESS, FAILURE}` (cpp:85, cpp:90)
//! as `Ok(())`/[`ResponseAdapterError::Failed`].
//!
//! # Not ported: `if (!res.trajectory)` (cpp:73-79)
//!
//! See [`crate::response::PlanningResponse`]'s "No `Option`" section —
//! `trajectory` is never absent in this port.
//!
//! # Ported computation
//!
//! `TimeOptimalTrajectoryGeneration totg(path_tolerance, resample_dt,
//! min_angle_change); totg.computeTimeStamps(*res.trajectory,
//! req.max_velocity_scaling_factor, req.max_acceleration_scaling_factor)`
//! (cpp:82-83) becomes
//! [`cspace_core::trajectory::trajectory_tools::apply_totg_time_parameterization`]
//! — already ported in `cspace_core::trajectory`, not re-implemented here.
//!
//! # History: this adapter no longer "closes" a STOMP gap
//!
//! An earlier round of this doc claimed this adapter closes a
//! `cspace_planners::stomp::conversion_functions::fill_robot_trajectory`
//! placeholder-`dt` gap: that function used to hand back a `RobotTrajectory`
//! with every waypoint after the first at an inert `dt = 0.1`, silently
//! wrong unless something downstream reparameterized it. That is no longer
//! how `fill_robot_trajectory` behaves — checked directly against
//! `crates/cspace-planners/src/stomp/conversion_functions.rs`, not assumed
//! from the old claim: it now returns `UnparameterizedTrajectory`, a type
//! with no duration accessor at all, and the only way to obtain a real
//! `RobotTrajectory` from it is `UnparameterizedTrajectory::into_uniformly_timed(dt)`,
//! which requires the caller to name `dt` explicitly. There is no longer a
//! placeholder `0.1` anywhere for a response adapter to overwrite — the fix
//! landed at the source, by making the silently-wrong-value path
//! unrepresentable, not by relying on a downstream consumer to catch it.
//!
//! [`AddTimeOptimalParameterization::adapt`]'s TOTG computation below is
//! unchanged and still real general-purpose behavior — reparameterizing
//! timing on whatever trajectory this crate's response chain sees, from any
//! planner — it just no longer plays a STOMP-specific corrective role. See
//! `totg_overwrites_a_uniform_placeholder_duration` for a
//! regression test that TOTG genuinely replaces a uniform, non-time-optimal
//! duration profile with a real one; it no longer references STOMP's own
//! (removed) placeholder shape.

use crate::scene::PlanningScene;
use cspace_collision::ParryCollisionEnv;
use cspace_core::trajectory::trajectory_tools::apply_totg_time_parameterization;

use crate::PlanningResponseAdapter;
use crate::error::ResponseAdapterError;
use crate::request::PlanningRequest;
use crate::response::PlanningResponse;

/// `TimeOptimalTrajectoryGeneration`'s three defaulted constructor
/// arguments. Replaces upstream's `default_response_adapter_parameters::ParamListener`
/// (D1 — see the module doc's "Deviation" note); a caller who would have set
/// these via a ROS parameter YAML file passes them to
/// [`AddTimeOptimalParameterization::new`] instead.
#[derive(Debug, Clone, Copy)]
pub struct AddTimeOptimalParameterization {
    path_tolerance: f64,
    resample_dt: f64,
    min_angle_change: f64,
}

impl AddTimeOptimalParameterization {
    /// See [`cspace_core::trajectory::time_optimal_trajectory_generation::TotgOptions`]
    /// for what each argument means; `TotgOptions::default()`'s values match
    /// upstream's own defaults.
    ///
    /// # `resample_dt` is not validated here, by design
    ///
    /// This matches upstream, not just defers to it by accident: the real
    /// `AddTimeOptimalParameterization::adapt` (cpp:68-92) has no persistent
    /// `resample_dt` field at all — it re-reads `params.resample_dt` from
    /// `ParamListener` fresh on every call (cpp:81) and passes it straight
    /// into `TimeOptimalTrajectoryGeneration`'s constructor (cpp:82), whose
    /// own body stores it unchecked (`time_optimal_trajectory_generation.cpp:918-920`).
    /// There is no upstream call site, at any layer, where an invalid
    /// `resample_dt` is rejected before use. `TotgOptions::with_resample_dt`
    /// (see [`cspace_core::trajectory::trajectory_tools::apply_totg_time_parameterization`]'s
    /// `# Errors` section) is a deviation *forward* of upstream, not a gap:
    /// it turns upstream's silent size_t-saturating cast into a real
    /// `Result` at the point the value is actually consumed. Rejecting here
    /// too, at construction, would validate a value this type does not yet
    /// know is destined for TOTG — [`Self`] is plain adapter configuration,
    /// matching upstream's "adapter has no ctor args, reads params at call
    /// time" shape as closely as a Rust type with stored fields can.
    pub fn new(path_tolerance: f64, resample_dt: f64, min_angle_change: f64) -> Self {
        Self {
            path_tolerance,
            resample_dt,
            min_angle_change,
        }
    }
}

impl PlanningResponseAdapter for AddTimeOptimalParameterization {
    fn description(&self) -> &'static str {
        "AddTimeOptimalParameterization"
    }

    fn adapt<'m>(
        &self,
        _scene: &mut PlanningScene<'m>,
        _env: &ParryCollisionEnv,
        request: &PlanningRequest,
        response: &mut PlanningResponse<'m>,
    ) -> Result<(), ResponseAdapterError> {
        apply_totg_time_parameterization(
            &mut response.trajectory,
            request.max_velocity_scaling_factor,
            request.max_acceleration_scaling_factor,
            self.path_tolerance,
            self.resample_dt,
            self.min_angle_change,
        )
        .map_err(|source| ResponseAdapterError::Failed {
            adapter: self.description(),
            source,
        })
    }
}

#[cfg(test)]
mod tests {
    use cspace_core::model::{MeshSearchPaths, RobotModel};
    use cspace_core::srdf::SrdfModel;
    use cspace_core::state::RobotState;
    use cspace_core::trajectory::RobotTrajectory;
    use cspace_core::trajectory::time_optimal_trajectory_generation::TotgOptions;
    use std::fs;

    use super::*;

    /// `panda.urdf` carries velocity limits but, like every URDF, no
    /// acceleration limits (URDF has no such element) — TOTG requires one
    /// per joint (`TimeOptimalTrajectoryGeneration` cannot compute a
    /// time-optimal profile against an unbounded axis). Fixed up the same
    /// way `cspace_core::trajectory`'s own `trajectory_tools`
    /// `set_uniform_acceleration_bound` test helper does: read each
    /// `panda_arm` joint's [`cspace_core::model::joint::JointModel::variable_bounds_msg`],
    /// set `has_acceleration_limits`/`max_acceleration`, write back via
    /// [`cspace_core::model::joint::JointModel::set_variable_bounds_from_limits`].
    fn panda() -> (RobotModel, SrdfModel) {
        let root = concat!(env!("CARGO_MANIFEST_DIR"), "/../../fixtures");
        let urdf_xml = fs::read_to_string(format!("{root}/panda.urdf")).unwrap();
        let urdf = urdf_rs::read_from_string(&urdf_xml).expect("fixture URDF must parse");
        let srdf = SrdfModel::parse_file(format!("{root}/panda.srdf")).unwrap();
        let mut model =
            RobotModel::from_urdf_and_srdf(&urdf, &urdf_xml, &srdf, &MeshSearchPaths::none())
                .expect("fixture model must build");
        for name in [
            "panda_joint1",
            "panda_joint2",
            "panda_joint3",
            "panda_joint4",
            "panda_joint5",
            "panda_joint6",
            "panda_joint7",
        ] {
            let joint = model.joint_model_mut(name).expect("panda_arm joint exists");
            let mut limits = joint.variable_bounds_msg();
            for limit in &mut limits {
                limit.has_acceleration_limits = true;
                limit.max_acceleration = 2.0;
            }
            joint.set_variable_bounds_from_limits(&limits);
        }
        (model, srdf)
    }

    fn request() -> PlanningRequest {
        PlanningRequest {
            group_name: "panda_arm".to_string(),
            goal_constraints: vec![],
            path_constraints: None,
            workspace_bounds: Default::default(),
            max_velocity_scaling_factor: 1.0,
            max_acceleration_scaling_factor: 1.0,
            ..Default::default()
        }
    }

    fn adapter() -> AddTimeOptimalParameterization {
        let d = TotgOptions::default();
        AddTimeOptimalParameterization::new(d.path_tolerance, d.resample_dt(), d.min_angle_change)
    }

    #[test]
    fn reparameterizes_a_two_waypoint_panda_arm_trajectory() {
        let (model, srdf) = panda();
        let mut scene = PlanningScene::new(&model, &srdf);
        let env = ParryCollisionEnv::default();

        let mut start = RobotState::new(&model);
        start.set_to_default_values();
        let mut goal = start.clone();
        goal.set_joint_positions("panda_joint1", &[0.4]).unwrap();

        let mut trajectory = RobotTrajectory::for_group_name(&model, "panda_arm").unwrap();
        let start_state = start.clone();
        trajectory.add_suffix_way_point(start, 0.0).unwrap();
        trajectory.add_suffix_way_point(goal, 0.0).unwrap();
        let mut response = PlanningResponse {
            start_state,
            trajectory,
            planner_id: String::new(),
        };

        assert_eq!(adapter().description(), "AddTimeOptimalParameterization");
        adapter()
            .adapt(&mut scene, &env, &request(), &mut response)
            .expect("a two-waypoint panda_arm move must reparameterize successfully");

        assert!(response.trajectory.duration() > 0.0);
    }

    /// See the module doc's "History" section — this no longer pins a live
    /// STOMP gap (round 21 closed that at the source), it is a general
    /// regression test that a uniform, non-time-optimal duration profile
    /// (waypoint 0 at `dt = 0.0`, every later waypoint at a uniform
    /// `dt = 0.1`, the shape any naive fixed-step discretization produces)
    /// gets overwritten with real time-optimal durations.
    #[test]
    fn totg_overwrites_a_uniform_placeholder_duration() {
        let (model, srdf) = panda();
        let mut scene = PlanningScene::new(&model, &srdf);
        let env = ParryCollisionEnv::default();

        let mut trajectory = RobotTrajectory::for_group_name(&model, "panda_arm").unwrap();
        let mut state = RobotState::new(&model);
        state.set_to_default_values();
        trajectory.add_suffix_way_point(state.clone(), 0.0).unwrap();
        for i in 1..6 {
            let mut waypoint = state.clone();
            waypoint
                .set_joint_positions("panda_joint1", &[0.05 * i as f64])
                .unwrap();
            // A uniform, non-time-optimal duration profile — the shape any
            // naive fixed-step discretization produces.
            trajectory.add_suffix_way_point(waypoint, 0.1).unwrap();
        }
        assert!(
            (1..trajectory.way_point_count())
                .all(|i| trajectory.way_point_duration_from_previous(i) == 0.1),
            "test setup must produce a uniform dt = 0.1 profile exactly"
        );

        let mut response = PlanningResponse {
            start_state: state,
            trajectory,
            planner_id: String::new(),
        };
        adapter()
            .adapt(&mut scene, &env, &request(), &mut response)
            .expect("a five-segment panda_arm move must reparameterize successfully");

        let still_uniform_placeholder = (1..response.trajectory.way_point_count())
            .all(|i| response.trajectory.way_point_duration_from_previous(i) == 0.1);
        assert!(
            !still_uniform_placeholder,
            "AddTimeOptimalParameterization must overwrite a uniform dt = 0.1 profile \
             with real time-optimal durations, not leave it in place"
        );
    }

    /// Pins the doc's "not validated here, by design" claim from the
    /// construction side: `new` must accept every one of these invalid
    /// `resample_dt` values without erroring or panicking.
    #[test]
    fn new_accepts_any_resample_dt_including_invalid_ones() {
        for resample_dt in [0.0, -1.0, f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
            let adapter = AddTimeOptimalParameterization::new(0.1, resample_dt, 0.001);
            assert_eq!(adapter.description(), "AddTimeOptimalParameterization");
        }
    }

    /// Pins the doc's "not validated here, by design" claim from the
    /// consuming side: an invalid `resample_dt` that survived `new`
    /// unchecked must be caught downstream, at `adapt` time, not silently
    /// saturate into the multi-gigabyte resample allocation the module doc
    /// describes.
    ///
    /// # Bite-check
    ///
    /// `adapt`'s single `.map_err` site (this file, `impl
    /// PlanningResponseAdapter for AddTimeOptimalParameterization::adapt`)
    /// wraps *any* `cspace_core::error::Error` `apply_totg_time_parameterization`
    /// returns into `ResponseAdapterError::Failed` — and that function
    /// reaches more than one such site: `TotgOptions::with_resample_dt`'s
    /// guard (`cspace-core/src/trajectory/time_optimal_trajectory_generation.rs:440-444`)
    /// for this test's inputs, but also `compute_time_stamps`'s own
    /// per-joint velocity/acceleration-limit checks a few lines further in
    /// the same function (`:492-526`, at least 4 more `Error::other` sites).
    /// The old assertion here, `matches!(err, ResponseAdapterError::Failed
    /// { .. })`, discards both `adapter` and `source` and so cannot tell
    /// "resample_dt was rejected" apart from any of those — swapping which
    /// guard fired would leave it green. `adapter` is already pinned by
    /// `PipelineError::Planner`-style tests elsewhere in this crate; `source`
    /// is what this test is actually supposed to pin, so the fix asserts on
    /// it via `err.to_string()` (`ResponseAdapterError::Failed`'s `#[error]`
    /// format includes `{source}`).
    ///
    /// `with_resample_dt`'s guard lives in `cspace_core::trajectory`, outside this
    /// round's fence, so the two-mutation standard's mutation (1) (no-op the
    /// guard, confirm the test fails) cannot be run by editing it directly.
    /// `sibling_cause_does_not_read_as_a_resample_dt_rejection` below is the
    /// substitute: a *real*, already-reachable second cause
    /// (`compute_time_stamps`'s missing-acceleration-limit check, hit by
    /// using the unpatched `panda_no_acceleration_limits` fixture rather than
    /// mutating any guard) stands in for the sibling branch and is asserted
    /// to reach `ResponseAdapterError::Failed` while *not* mentioning
    /// `resample_dt` — proving `err.to_string().contains("resample_dt")`
    /// below actually discriminates the two causes, not just "some Failed".
    #[test]
    fn adapt_rejects_an_invalid_resample_dt_deferred_from_new() {
        let (model, srdf) = panda();
        let mut scene = PlanningScene::new(&model, &srdf);
        let env = ParryCollisionEnv::default();

        let mut start = RobotState::new(&model);
        start.set_to_default_values();
        let mut goal = start.clone();
        goal.set_joint_positions("panda_joint1", &[0.4]).unwrap();

        let mut trajectory = RobotTrajectory::for_group_name(&model, "panda_arm").unwrap();
        let start_state = start.clone();
        trajectory.add_suffix_way_point(start, 0.0).unwrap();
        trajectory.add_suffix_way_point(goal, 0.0).unwrap();
        let mut response = PlanningResponse {
            start_state,
            trajectory,
            planner_id: String::new(),
        };

        let d = TotgOptions::default();
        for resample_dt in [0.0, -1.0, f64::NAN] {
            let bad = AddTimeOptimalParameterization::new(
                d.path_tolerance,
                resample_dt,
                d.min_angle_change,
            );
            let err = bad
                .adapt(&mut scene, &env, &request(), &mut response)
                .expect_err("an invalid resample_dt must be rejected at adapt time, not accepted");
            assert!(
                matches!(err, ResponseAdapterError::Failed { .. }),
                "resample_dt = {resample_dt} must fail as ResponseAdapterError::Failed, got {err:?}"
            );
            let message = err.to_string();
            assert!(
                message.contains("resample_dt"),
                "resample_dt = {resample_dt} must be rejected by the resample_dt guard \
                 specifically, not by some other ResponseAdapterError::Failed cause: {message}"
            );
        }
    }

    /// The sibling half of `adapt_rejects_an_invalid_resample_dt_deferred_from_new`'s
    /// bite-check (see its doc comment): a *different*, already-reachable
    /// cause of `ResponseAdapterError::Failed` — a `panda_arm` whose joints
    /// carry no acceleration limits, the ordinary URDF case per this
    /// module's `panda()` fixture doc — must still fail, but its message
    /// must NOT contain `resample_dt`. If it did, the sibling test above's
    /// `contains("resample_dt")` check would not actually be discriminating
    /// between the two causes.
    #[test]
    fn sibling_cause_does_not_read_as_a_resample_dt_rejection() {
        let root = concat!(env!("CARGO_MANIFEST_DIR"), "/../../fixtures");
        let urdf_xml = fs::read_to_string(format!("{root}/panda.urdf")).unwrap();
        let urdf = urdf_rs::read_from_string(&urdf_xml).expect("fixture URDF must parse");
        let srdf = SrdfModel::parse_file(format!("{root}/panda.srdf")).unwrap();
        let model =
            RobotModel::from_urdf_and_srdf(&urdf, &urdf_xml, &srdf, &MeshSearchPaths::none())
                .expect("fixture model must build");
        let mut scene = PlanningScene::new(&model, &srdf);
        let env = ParryCollisionEnv::default();

        let mut start = RobotState::new(&model);
        start.set_to_default_values();
        let mut goal = start.clone();
        goal.set_joint_positions("panda_joint1", &[0.4]).unwrap();

        let mut trajectory = RobotTrajectory::for_group_name(&model, "panda_arm").unwrap();
        let start_state = start.clone();
        trajectory.add_suffix_way_point(start, 0.0).unwrap();
        trajectory.add_suffix_way_point(goal, 0.0).unwrap();
        let mut response = PlanningResponse {
            start_state,
            trajectory,
            planner_id: String::new(),
        };

        let err = adapter()
            .adapt(&mut scene, &env, &request(), &mut response)
            .expect_err(
                "a panda_arm with no acceleration limits must be rejected by \
                 compute_time_stamps, not silently accepted",
            );
        assert!(
            matches!(err, ResponseAdapterError::Failed { .. }),
            "expected ResponseAdapterError::Failed, got {err:?}"
        );
        let message = err.to_string();
        assert!(
            !message.contains("resample_dt"),
            "a missing-acceleration-limit rejection must not read as a resample_dt \
             rejection: {message}"
        );
    }
}
