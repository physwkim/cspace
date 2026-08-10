#!/usr/bin/env python3
# Copyright (c) 2026, moveit-rs contributors
# SPDX-License-Identifier: BSD-3-Clause
"""Regenerates crates/cspace-collision/tests/fixtures/<robot>_collision.json.

Ground-truth capture for the oracle's `collision` op (tools/moveit-oracle/src/
oracle.cpp): one default case (every joint at its default value) plus three
cases at oracle-drawn random joint positions (`random_states`, seeded for
reproducibility), each checked against a single fixed world object -- a
4x4x0.1 floor box centered at (0, 0, -0.05), so its top face sits at z = 0 --
matching `crates/cspace-collision/tests/collision_parity.rs`'s own
`floor_env`/`build_acm`.

Run after rebuilding the oracle image (tools/moveit-oracle/build.sh):

    sg docker -c 'python3 tools/moveit-oracle/capture-collision-fixtures.py'

fanuc_collision.json and pr2_collision.json already existed; this script
regenerates them and adds panda_collision.json, which did not exist before
because `cspace-model` did not load `<mesh>` collision geometry at all --
every panda link's collision geometry is exactly one `<mesh>` element, so a
capture against the old rust-side (geometry-free) build would have compared
nothing to nothing. The existing fanuc_collision.json was itself captured
before tools/moveit-oracle/Dockerfile's `MOVEIT2_PACKAGES` included
`moveit_resources_fanuc_description`: the oracle's own RobotModel silently
built with zero fanuc collision links too (every case's distance is exactly
f64::MAX in the old fixture), so that fixture was not a disagreement-free
ground truth, just an accidental agreement between two geometry-free sides.
Regenerating it now, against an oracle that can actually resolve fanuc's
package, is required for `collision_parity.rs` to mean anything for fanuc.
"""
import json
import pathlib
import subprocess
import sys

REPO_ROOT = pathlib.Path(__file__).resolve().parents[2]
RUN_ORACLE = REPO_ROOT / "tools" / "moveit-oracle" / "run-oracle.sh"
FIXTURES_DIR = REPO_ROOT / "crates" / "cspace-collision" / "tests" / "fixtures"

SEED = 20260803
RANDOM_CASES = 3

# The 4x4x0.1 floor box at (0, 0, -0.05), identity rotation, row-major 4x4.
FLOOR_OBJECT = {
    "id": "floor",
    "pose": [
        1.0, 0.0, 0.0, 0.0,
        0.0, 1.0, 0.0, 0.0,
        0.0, 0.0, 1.0, -0.05,
        0.0, 0.0, 0.0, 1.0,
    ],
    "shape": {"type": "box", "size": [4.0, 4.0, 0.1]},
}

ROBOTS = ["panda", "fanuc", "pr2"]


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


def collision_case(oracle, joint_values):
    result = oracle.ask(
        op="collision",
        joint_values=joint_values,
        objects=[FLOOR_OBJECT],
    )
    return {"joint_values": joint_values, **result}


def capture(robot):
    urdf = REPO_ROOT / "fixtures" / f"{robot}.urdf"
    srdf = REPO_ROOT / "fixtures" / f"{robot}.srdf"
    oracle = Oracle(urdf, srdf)
    try:
        cases = [collision_case(oracle, {})]
        states = oracle.ask(op="random_states", count=RANDOM_CASES, seed=SEED)["states"]
        for state in states:
            cases.append(collision_case(oracle, state))
        return {"cases": cases}
    finally:
        oracle.close()


def main():
    FIXTURES_DIR.mkdir(parents=True, exist_ok=True)
    for robot in ROBOTS:
        print(f"capturing {robot}...", file=sys.stderr)
        data = capture(robot)
        out = FIXTURES_DIR / f"{robot}_collision.json"
        out.write_text(json.dumps(data, indent=2, sort_keys=True) + "\n")
        print(f"wrote {out}", file=sys.stderr)


if __name__ == "__main__":
    main()
