// Copyright (c) 2023, PickNik Inc.
// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// Ported from moveit2 @ e017c91ee12984393a28ba246075c65f69cde3bf:
//   moveit_ros/planning/planning_pipeline_interfaces/src/plan_responses_container.cpp
//   moveit_ros/planning/planning_pipeline_interfaces/src/solution_selection_functions.cpp
//   moveit_ros/planning/planning_pipeline_interfaces/src/stopping_criterion_function.cpp
//
// Not ported from the same directory: `planning_pipeline_interfaces.cpp`'s
// `getLogger`/`createPlanningPipelineMap` (D1: `rclcpp` logging and a
// `rclcpp::Node::SharedPtr` parameter) and `planWithSinglePipeline`/
// `planWithParallelPipelines` (both resolve a request by `pipeline_id`
// against a name-keyed `unordered_map<string, PlanningPipelinePtr>` this
// crate does not build -- the same deferred-to-a-registry reasoning
// `crate::request`'s own doc already gives for leaving `PlanningRequest`
// without a `pipeline_id` field: "selects *among* pipelines, a
// caller/orchestration concern, and this workspace has exactly one
// pipeline"). See `doc/claim-audit/moveit-planning.md` for the full
// per-file verdict.

//! Collecting and choosing among multiple [`crate::pipeline::generate_plan`]
//! outcomes -- the parts of `planning_pipeline_interfaces` that do not need
//! a concrete, name-keyed pipeline map to be useful.
//!
//! # `PlanOutcome` replaces a nullable-trajectory `MotionPlanResponse`
//!
//! Upstream's `PlanResponsesContainer`/`getShortestSolution`/
//! `stopAtFirstSolution` all operate on
//! `::planning_interface::MotionPlanResponse`, which can be "successful"
//! (`trajectory` set, `bool(solution) == true`) or "failed" (`trajectory`
//! null, an error code set) while remaining a single non-`Option` type.
//! [`crate::response::PlanningResponse`] dropped that nullable case (see its
//! own doc, "No `Option`") because [`crate::pipeline::generate_plan`]
//! already reports failure through `Result`'s `Err`, not a null field. So
//! [`PlanOutcome`] -- what this module's container actually stores -- is
//! that same `Result<PlanningResponse, PipelineError>`, not a bare
//! `PlanningResponse`: this is the same success/failure distinction
//! upstream's nullable trajectory carries, expressed the way the rest of
//! this crate already expresses it.
//!
//! # `shortest_solution` does not reproduce upstream's empty-input UB
//!
//! `getShortestSolution` calls `std::min_element` and dereferences the
//! result unconditionally; on an empty `solutions` vector, `min_element`
//! returns `end()` and that dereference is undefined behavior. This port
//! returns `Option<&PlanOutcome>`, `None` for that case, rather than
//! reproducing it -- a typed empty case, not a panic or a fabricated
//! element.
//!
//! For a non-empty input the tie-break matches upstream's comparator
//! exactly: a successful [`PlanOutcome`] always beats a failed one
//! regardless of path length (`solution_a && solution_b` false-branches of
//! the C++ comparator), and among two successes the shorter
//! [`moveit_trajectory::robot_trajectory::path_length`] wins. On a tie --
//! equal path length, or an input containing only failures -- the *first*
//! candidate wins: `std::min_element`'s comparator only replaces the
//! running best on a *strict* improvement, so an equal or non-improving
//! candidate never displaces an earlier one. [`shortest_solution`] below
//! implements the same strict-improvement fold rather than delegating to
//! [`Iterator::min_by`], whose first-vs-last tie-break is a different
//! contract that should not be trusted to happen to match by accident.

use std::sync::{Mutex, MutexGuard};

use moveit_trajectory::robot_trajectory::path_length;

use crate::pipeline::PipelineError;
use crate::request::PlanningRequest;
use crate::response::PlanningResponse;

/// One [`crate::pipeline::generate_plan`] result, as stored by
/// [`PlanResponsesContainer`]. See the module doc, "`PlanOutcome` replaces a
/// nullable-trajectory `MotionPlanResponse`".
pub type PlanOutcome<'m> = Result<PlanningResponse<'m>, PipelineError>;

/// Replaces `PlanResponsesContainer`: a mutex-guarded vector multiple
/// planning threads can push a [`PlanOutcome`] into concurrently.
#[derive(Debug)]
pub struct PlanResponsesContainer<'m> {
    solutions: Mutex<Vec<PlanOutcome<'m>>>,
}

impl<'m> PlanResponsesContainer<'m> {
    /// `PlanResponsesContainer(expected_size)`.
    pub fn new(expected_size: usize) -> Self {
        Self {
            solutions: Mutex::new(Vec::with_capacity(expected_size)),
        }
    }

    /// `pushBack`.
    pub fn push_back(&self, outcome: PlanOutcome<'m>) {
        self.solutions
            .lock()
            .expect("PlanResponsesContainer mutex poisoned by a panicking planning thread")
            .push(outcome);
    }

    /// `getSolutions`.
    pub fn solutions(&self) -> MutexGuard<'_, Vec<PlanOutcome<'m>>> {
        self.solutions
            .lock()
            .expect("PlanResponsesContainer mutex poisoned by a panicking planning thread")
    }
}

/// `getShortestSolution`. See the module doc, "`shortest_solution` does not
/// reproduce upstream's empty-input UB", for the empty-input and tie-break
/// behavior.
pub fn shortest_solution<'a, 'm>(solutions: &'a [PlanOutcome<'m>]) -> Option<&'a PlanOutcome<'m>> {
    let mut best: Option<&'a PlanOutcome<'m>> = None;
    for candidate in solutions {
        let replace = match best {
            None => true,
            Some(current) => is_strictly_shorter(candidate, current),
        };
        if replace {
            best = Some(candidate);
        }
    }
    best
}

/// The comparator `getShortestSolution` passes to `std::min_element`: `a` is
/// preferred over `b` exactly when both are `Ok` and `a`'s path is
/// strictly shorter, or only `a` is `Ok`. Anything else -- both failed, or
/// only `b` succeeded -- is `false`, matching the C++ `else return false;`
/// fallthrough.
fn is_strictly_shorter(a: &PlanOutcome<'_>, b: &PlanOutcome<'_>) -> bool {
    match (a, b) {
        (Ok(ra), Ok(rb)) => path_length(&ra.trajectory) < path_length(&rb.trajectory),
        (Ok(_), Err(_)) => true,
        (Err(_), _) => false,
    }
}

/// `stopAtFirstSolution`. `plan_requests` is unused, matching upstream's own
/// unused `/*plan_requests*/` parameter.
pub fn stop_at_first_solution<'m>(
    plan_responses_container: &PlanResponsesContainer<'m>,
    _plan_requests: &[PlanningRequest],
) -> bool {
    plan_responses_container
        .solutions()
        .iter()
        .any(|solution| solution.is_ok())
}

#[cfg(test)]
mod tests {
    use moveit_model::{MeshSearchPaths, RobotModel};
    use moveit_srdf::SrdfModel;
    use moveit_state::RobotState;
    use moveit_trajectory::RobotTrajectory;
    use std::fs;

    use super::*;
    use crate::pipeline::PipelineError;

    fn panda() -> (RobotModel, SrdfModel) {
        let root = concat!(env!("CARGO_MANIFEST_DIR"), "/../../fixtures");
        let urdf_xml = fs::read_to_string(format!("{root}/panda.urdf")).unwrap();
        let urdf = urdf_rs::read_from_string(&urdf_xml).expect("fixture URDF must parse");
        let srdf = SrdfModel::parse_file(format!("{root}/panda.srdf")).unwrap();
        let model =
            RobotModel::from_urdf_and_srdf(&urdf, &urdf_xml, &srdf, &MeshSearchPaths::none())
                .expect("fixture model must build");
        (model, srdf)
    }

    /// A [`PlanningResponse`] whose trajectory has exactly one non-zero-length
    /// segment: `panda_joint1` moved by `delta` radians, every other joint
    /// held fixed. Two waypoints at different `delta`s have strictly
    /// different, comparable path lengths.
    fn response_with_path_length<'m>(model: &'m RobotModel, delta: f64) -> PlanningResponse<'m> {
        let mut start = RobotState::new(model);
        start.set_to_default_values();
        let start_state = start.clone();
        let mut goal = start.clone();
        goal.set_joint_positions("panda_joint1", &[delta]).unwrap();

        let mut trajectory = RobotTrajectory::for_group_name(model, "panda_arm").unwrap();
        trajectory.add_suffix_way_point(start, 0.0).unwrap();
        trajectory.add_suffix_way_point(goal, 0.0).unwrap();

        PlanningResponse {
            trajectory,
            planner_id: String::new(),
            start_state,
        }
    }

    fn failure() -> PipelineError {
        PipelineError::NoPlanners
    }

    #[test]
    fn plan_responses_container_returns_pushed_outcomes_in_push_order() {
        let (model, _srdf) = panda();
        let container = PlanResponsesContainer::new(0);
        container.push_back(Ok(response_with_path_length(&model, 0.1)));
        container.push_back(Err(failure()));

        let solutions = container.solutions();
        assert_eq!(solutions.len(), 2);
        assert!(solutions[0].is_ok());
        assert!(solutions[1].is_err());
    }

    #[test]
    fn shortest_solution_is_none_on_empty_input() {
        let solutions: Vec<PlanOutcome> = vec![];
        assert!(shortest_solution(&solutions).is_none());
    }

    #[test]
    fn shortest_solution_picks_the_shorter_path_regardless_of_position() {
        let (model, _srdf) = panda();
        let short = Ok(response_with_path_length(&model, 0.1));
        let long = Ok(response_with_path_length(&model, 0.5));

        let solutions = vec![long, short];
        let picked = shortest_solution(&solutions).unwrap().as_ref().unwrap();
        assert_eq!(
            path_length(&picked.trajectory),
            path_length(&solutions[1].as_ref().unwrap().trajectory)
        );
    }

    #[test]
    fn shortest_solution_prefers_any_success_over_any_failure() {
        let (model, _srdf) = panda();
        let solutions = vec![Err(failure()), Ok(response_with_path_length(&model, 5.0))];
        assert!(shortest_solution(&solutions).unwrap().is_ok());
    }

    #[test]
    fn shortest_solution_keeps_the_first_candidate_on_an_exact_tie() {
        let (model, _srdf) = panda();
        let first = response_with_path_length(&model, 0.3);
        let first_path_length = path_length(&first.trajectory);
        let second = response_with_path_length(&model, -0.3);
        assert_eq!(
            first_path_length,
            path_length(&second.trajectory),
            "test setup: both waypoints must move panda_joint1 by the same magnitude"
        );

        let solutions = vec![Ok(first), Ok(second)];
        let picked = shortest_solution(&solutions).unwrap();
        assert!(std::ptr::eq(picked, &solutions[0]));
    }

    #[test]
    fn shortest_solution_keeps_the_first_candidate_when_every_candidate_failed() {
        let solutions = vec![Err(failure()), Err(failure())];
        let picked = shortest_solution(&solutions).unwrap();
        assert!(std::ptr::eq(picked, &solutions[0]));
    }

    #[test]
    fn stop_at_first_solution_is_false_on_an_empty_container() {
        let container = PlanResponsesContainer::new(0);
        assert!(!stop_at_first_solution(&container, &[]));
    }

    #[test]
    fn stop_at_first_solution_is_false_while_every_outcome_is_a_failure() {
        let container = PlanResponsesContainer::new(0);
        container.push_back(Err(failure()));
        container.push_back(Err(failure()));
        assert!(!stop_at_first_solution(&container, &[]));
    }

    #[test]
    fn stop_at_first_solution_is_true_once_any_outcome_succeeds() {
        let (model, _srdf) = panda();
        let container = PlanResponsesContainer::new(0);
        container.push_back(Err(failure()));
        container.push_back(Ok(response_with_path_length(&model, 0.2)));
        assert!(stop_at_first_solution(&container, &[]));
    }
}
