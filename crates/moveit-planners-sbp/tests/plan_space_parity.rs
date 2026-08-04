// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause

//! Phase 7 kickoff: a bit-level parity test against the oracle's `plan` op
//! (`32114d5`, PORTING-PLAN.md §118), pinning that
//! [`JointModelGroupSpace`]'s construction agrees with the OMPL
//! `CompoundStateSpace` `buildPlanSpace` builds beside it in the same
//! process.
//!
//! Phase 7's completion condition (§5) needs a path-length ratio against
//! C++ OMPL RRTConnect, and that ratio is meaningless unless both lengths
//! are measured in the same metric. Planner output itself is seed-dependent
//! (`§118.7`'s `seed_applied` caveat), so `distance_probes` -- the same
//! `a`/`b` joint maps fed to both sides' `space->distance`/`space.distance`
//! -- is the only surface on which the two constructions can be asserted
//! bit-identical at all. This is that assertion, not yet the planner-parity
//! benchmark itself (`§5`'s 500-problem, success-rate/path-length
//! conditions) -- see PORTING-PLAN.md's own round notes for what remains.
//!
//! Ground truth is `tests/fixtures/panda_arm_plan_distance_probes_response.json`,
//! captured verbatim from the oracle's `plan` response for the request
//! committed alongside it (`panda_arm_plan_distance_probes_request.json`),
//! registered in `tests/fixtures/oracle-models.json` per
//! `tools/ci/verify-fixture-replay.sh`. `panda_arm` is bounded-revolute-only
//! (`joint_model_group_space.rs`'s own `GROUPS` doc comment) -- the other
//! joint kinds (continuous, planar, prismatic, floating) are not yet
//! exercised by this parity surface and are named here as a gap, not
//! silently covered.

use std::collections::BTreeMap;
use std::fs;

use serde::Deserialize;

use moveit_model::{MeshSearchPaths, RobotModel};
use moveit_planners_sbp::{JointModelGroupSpace, StateSpace};
use moveit_srdf::SrdfModel;
use moveit_state::RobotState;

fn fixture_path(file_name: &str) -> String {
    format!(
        concat!(env!("CARGO_MANIFEST_DIR"), "/tests/fixtures/{}"),
        file_name
    )
}

#[derive(Deserialize)]
struct DistanceProbeRequest {
    a: BTreeMap<String, f64>,
    b: BTreeMap<String, f64>,
}

#[derive(Deserialize)]
struct PlanRequest {
    group: String,
    distance_probes: Vec<DistanceProbeRequest>,
}

#[derive(Deserialize)]
struct DistanceProbeResult {
    distance: f64,
}

#[derive(Deserialize)]
struct PlanResult {
    distance_probes: Vec<DistanceProbeResult>,
}

#[derive(Deserialize)]
struct PlanResponseEnvelope {
    result: PlanResult,
}

fn robot_state_from<'a>(
    model: &'a RobotModel,
    joint_values: &BTreeMap<String, f64>,
) -> RobotState<'a> {
    let mut state = RobotState::new(model);
    state.set_to_default_values();
    for (name, &value) in joint_values {
        state
            .set_variable_position(name, value)
            .unwrap_or_else(|e| panic!("set_variable_position({name}, {value}): {e}"));
    }
    state
}

/// Every `distance_probes[i].distance` the oracle's `plan` op reports for
/// `panda_arm` must equal, bit for bit, what this port's own
/// `JointModelGroupSpace::distance` computes for the same `a`/`b` states --
/// not just agree within a tolerance. Both sides sum the same per-joint
/// `1/(upper-lower)`-weighted terms in the same `getActiveJointModels()`
/// order (`buildPlanSpace`'s own doc comment in `oracle.cpp`), so nothing
/// here should be leaving room for summation-order or rounding slack; an
/// inequality would be a real disagreement, not noise to loosen the
/// assertion around.
#[test]
fn panda_arm_distance_probes_match_the_oracle_plan_space_bit_for_bit() {
    let request: Vec<PlanRequest> = serde_json::from_str(
        &fs::read_to_string(fixture_path("panda_arm_plan_distance_probes_request.json"))
            .expect("read request fixture"),
    )
    .expect("parse request fixture");
    let response: Vec<PlanResponseEnvelope> = serde_json::from_str(
        &fs::read_to_string(fixture_path("panda_arm_plan_distance_probes_response.json"))
            .expect("read response fixture"),
    )
    .expect("parse response fixture");

    let request = &request[0];
    let response = &response[0];
    assert_eq!(
        request.distance_probes.len(),
        response.result.distance_probes.len(),
        "request/response fixture have mismatched probe counts"
    );

    let urdf_path = fixture_path("panda.urdf");
    let srdf_path = fixture_path("panda.srdf");
    let urdf_xml = fs::read_to_string(&urdf_path).expect("read panda.urdf");
    let urdf = urdf_rs::read_file(&urdf_path).expect("parse panda.urdf");
    let srdf = SrdfModel::parse_file(&srdf_path).expect("parse panda.srdf");
    let model = RobotModel::from_urdf_and_srdf(&urdf, &urdf_xml, &srdf, &MeshSearchPaths::none())
        .expect("build panda RobotModel");

    let space = JointModelGroupSpace::new(&model, &request.group)
        .unwrap_or_else(|e| panic!("JointModelGroupSpace::new({}): {e}", request.group));

    for (i, (probe, expected)) in request
        .distance_probes
        .iter()
        .zip(&response.result.distance_probes)
        .enumerate()
    {
        let state_a = robot_state_from(&model, &probe.a);
        let state_b = robot_state_from(&model, &probe.b);
        let a = space.read_robot_state(&state_a);
        let b = space.read_robot_state(&state_b);
        let actual = space.distance(&a, &b);
        assert_eq!(
            actual, expected.distance,
            "probe {i}: JointModelGroupSpace::distance = {actual}, oracle plan.distance_probes[{i}].distance = {}",
            expected.distance
        );
    }
}
