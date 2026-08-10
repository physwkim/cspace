#!/usr/bin/env python3
# Copyright (c) 2026, moveit-rs contributors
# SPDX-License-Identifier: BSD-3-Clause
"""Regenerates crates/cspace-core/tests/fixtures/state/<robot>_dynamics.json.

Ground-truth capture for `dynamics_solver::DynamicsSolver` via the oracle's
`dynamics` op (tools/moveit-oracle/src/oracle.cpp). No Rust `cspace_core::state`
dynamics port exists yet -- `KDL::ChainIdSolver_RNE`'s own `.cpp` is not
present anywhere local to this repo, only its compiled `.so` and header
declaration -- so there is nothing to run a live differential comparison
against. This script exists to capture ground truth to plan and later verify
a port against, not to replace one.

Each robot gets `RANDOM_CASES` general cases at oracle-drawn random joint
positions (`random_states`, seeded for reproducibility) with fixed nonzero
velocity/acceleration, plus two cases reusing the first random case's
position that are derivable without any oracle at all:

  - "gravity_compensation": zero velocity and acceleration, real gravity --
    reduces to gravity compensation alone.
  - "zero_gravity": zero velocity, acceleration and gravity -- torque must
    come back exactly zero on every joint.

Velocity and acceleration have no URDF-declared bounds the way position
does, so they are not oracle-drawn: each is a small, distinct, deterministic
value per joint (arbitrary but fixed, so a rerun reproduces byte-identical
fixtures).

Run after rebuilding the oracle image (tools/moveit-oracle/build.sh):

    sg docker -c 'python3 tools/moveit-oracle/capture-dynamics-fixtures.py'

Two things the captured numbers do NOT mean, confirmed against the pinned
moveit2 checkout and this repo's own fixture URDFs before trusting them:

  - panda/fanuc/dual_arm_panda's `torques` come back exactly zero in every
    case, including the nonzero-velocity/acceleration ones: those three
    fixture URDFs (fixtures/{panda,fanuc,dual_arm_panda}.urdf) have no
    `<inertial>` element on any link at all, so every body in the chain is
    massless to KDL. This is not a solver defect, and not a `cspace_core::model`
    gap either -- `DynamicsSolver` reads mass/inertia from the raw URDF via
    `kdl_parser`, bypassing `moveit::core::RobotModel`/`LinkModel` entirely,
    which is why the oracle answers this op with no Rust-side model at all.
    Only pr2.urdf carries real `<inertial>` data among the four fixtures.
  - pr2_dynamics.json's `max_payload.payload` is `0.0` in every case. This
    is `DynamicsSolver::getMaxPayload` itself: its `max_torques_` is built
    over *every* group joint name (fixed included, `0.0` for each), but the
    saturation loop reads it with the active-joint-only index `getTorques`
    uses -- see oracle.cpp's `dynamics()` doc comment. `right_arm` has a
    fixed joint (`r_upper_arm_joint`) before its last active joint
    (`r_elbow_flex_joint`), so the loop ends up comparing
    `r_elbow_flex_joint`'s real gravity torque against `r_upper_arm_joint`'s
    always-`0.0` slot. Confirmed against
    moveit_core/dynamics_solver/src/dynamics_solver.cpp directly, not
    inferred from the output alone.
"""
import json
import pathlib
import subprocess
import sys

REPO_ROOT = pathlib.Path(__file__).resolve().parents[2]
RUN_ORACLE = REPO_ROOT / "tools" / "moveit-oracle" / "run-oracle.sh"
FIXTURES_DIR = REPO_ROOT / "crates" / "cspace_core::state" / "tests" / "fixtures"

GRAVITY = [0.0, 0.0, -9.81]
PAYLOAD_KG = 1.0
SEED = 20260803
RANDOM_CASES = 5

# One representative chain group per fixture robot -- a chain with no mimic
# joint, DynamicsSolver's own two hard requirements. pr2's gripper mimic
# joints and dual_arm_panda's finger mimic joints both live outside their
# robot's arm group, same as panda's own hand/mimic split.
ROBOTS = [
    ("panda", "panda_arm"),
    ("fanuc", "manipulator"),
    ("dual_arm_panda", "left_panda_arm"),
    ("pr2", "right_arm"),
]


class Oracle:
    def __init__(self, urdf, srdf):
        self.proc = subprocess.Popen(
            [str(RUN_ORACLE), "--urdf", str(urdf), "--srdf", str(srdf)],
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=sys.stderr,
            text=True,
            bufsize=1,
        )
        self.next_id = 0

    def ask(self, **op):
        req = {"id": self.next_id, **op}
        self.next_id += 1
        self.proc.stdin.write(json.dumps(req) + "\n")
        self.proc.stdin.flush()
        line = self.proc.stdout.readline()
        if not line:
            raise RuntimeError(f"oracle closed stdout answering {req}")
        resp = json.loads(line)
        if not resp["ok"]:
            raise RuntimeError(f"oracle error for {req}: {resp.get('error')}")
        return resp["result"]

    def close(self):
        self.proc.stdin.close()
        self.proc.wait()


def zeros_map(names):
    return {n: 0.0 for n in names}


def deterministic_velocities(names):
    return {n: 0.1 * (i + 1) * (-1 if i % 2 else 1) for i, n in enumerate(names)}


def deterministic_accelerations(names):
    return {n: 0.05 * (i + 1) * (-1 if i % 2 == 0 else 1) for i, n in enumerate(names)}


def dynamics_case(oracle, name, group, gravity, joint_values, joint_velocities,
                   joint_accelerations, payload):
    result = oracle.ask(
        op="dynamics",
        group=group,
        gravity=gravity,
        joint_values=joint_values,
        joint_velocities=joint_velocities,
        joint_accelerations=joint_accelerations,
        payload=payload,
    )
    return {
        "name": name,
        "gravity": gravity,
        "joint_values": joint_values,
        "joint_velocities": joint_velocities,
        "joint_accelerations": joint_accelerations,
        "payload": payload,
        **result,
    }


def capture(robot, group):
    urdf = REPO_ROOT / "fixtures" / f"{robot}.urdf"
    srdf = REPO_ROOT / "fixtures" / f"{robot}.srdf"
    oracle = Oracle(urdf, srdf)
    try:
        model = oracle.ask(op="model_info")
        joint_type = {j["name"]: j["type_name"] for j in model["joint_details"]}
        active_names = [n for n in model["groups"][group] if joint_type[n] != "Fixed"]

        states = oracle.ask(op="random_states", count=RANDOM_CASES, seed=SEED)["states"]

        cases = []
        for i, state in enumerate(states):
            joint_values = {n: state[n] for n in active_names}
            cases.append(dynamics_case(
                oracle, f"random_{i}", group, GRAVITY,
                joint_values, deterministic_velocities(active_names),
                deterministic_accelerations(active_names), PAYLOAD_KG,
            ))

        base_values = cases[0]["joint_values"]
        cases.append(dynamics_case(
            oracle, "gravity_compensation", group, GRAVITY,
            base_values, zeros_map(active_names), zeros_map(active_names), 0.0,
        ))
        cases.append(dynamics_case(
            oracle, "zero_gravity", group, [0.0, 0.0, 0.0],
            base_values, zeros_map(active_names), zeros_map(active_names), 0.0,
        ))

        return {"robot": robot, "group": group, "cases": cases}
    finally:
        oracle.close()


def main():
    FIXTURES_DIR.mkdir(parents=True, exist_ok=True)
    for robot, group in ROBOTS:
        print(f"capturing {robot}/{group}...", file=sys.stderr)
        data = capture(robot, group)
        out = FIXTURES_DIR / f"{robot}_dynamics.json"
        out.write_text(json.dumps(data, indent=2, sort_keys=True) + "\n")
        print(f"wrote {out}", file=sys.stderr)


if __name__ == "__main__":
    main()
