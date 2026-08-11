#!/usr/bin/env python3
# Copyright (c) 2026, cspace contributors
# SPDX-License-Identifier: BSD-3-Clause
"""Captures the oracle's `ccd` op into crates/cspace-collision/tests/fixtures.

`ccd` is `CollisionEnvBullet::checkRobotCollision(req, res, state1, state2,
acm)` -- the two-state swept check, and the only MoveIt query answered by a
continuous algorithm. `crates/cspace-collision/tests/ccd_parity.rs` replays
every case captured here through `ParryCollisionEnv::
check_robot_collision_continuous`, which is this workspace's port of the same
function down to bullet's own GJK/EPA.

The state *pairs* are what makes this op's fixtures different from
`capture-collision-fixtures.py`'s. A swept check only reaches the interesting
code -- `CastHullShape`'s support function, `addCastSingleResult`'s
`percent_interpolation` -- when the two states actually differ, so the cases
here are consecutive pairs drawn from one `random_states` batch, plus a
default-to-first pair. Sampling each pair independently would give the same
coverage at twice the oracle round trips and no extra information: what the
sweep sees is a function of the two endpoints, not of how they were drawn.

The world is the same 4x4x0.1 floor box at the same pose the discrete fixtures
use, plus a pillar the arms actually sweep through (see `PILLAR_OBJECT`).
`ccd_parity.rs` builds both with the same numbers a third time, which is why
they are stated as one constant per side rather than shared through a file
neither language reads.

Run from a clean checkout (the oracle image is content-stamped; see
`run-oracle.sh`):

    sg docker -c tools/moveit-oracle/capture-ccd-fixtures.py
"""
import json
import pathlib
import subprocess
import sys

REPO_ROOT = pathlib.Path(__file__).resolve().parents[2]
RUN_ORACLE = REPO_ROOT / "tools" / "moveit-oracle" / "run-oracle.sh"
FIXTURES_DIR = REPO_ROOT / "crates" / "cspace-collision" / "tests" / "fixtures"

# A seed of this file's own, not `capture-collision-fixtures.py`'s: reusing
# that one would make every state here a state the discrete fixtures already
# cover, so the two files would sample the same corner of configuration space
# and the sweep between them would be the only thing not already tested.
SEED = 20260810
RANDOM_CASES = 12

# The 4x4x0.1 floor box at (0, 0, -0.05), identity rotation, row-major 4x4 --
# the same object `capture-collision-fixtures.py` uses, so its top face sits
# at z = 0.
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

# A 0.2x0.2x1.5 pillar standing at (0.5, 0, 0.75), i.e. resting on the floor
# directly in front of the arm.
#
# The floor alone is not enough to exercise a swept check. It is only ever met
# by the base and the wheels, which barely move between two sampled states --
# and on pr2 it is not met at all: every one of `pr2_collision.json`'s cases
# reports a 4.4mm gap to it, so a pr2 fixture built from the floor alone holds
# twelve cases that all say `collision: false` and compare two empty contact
# maps. The pillar stands where all three arms reach, so a sweep between two
# sampled states crosses it and the fixture carries contacts whose `depth`,
# `normal`, `pos` and `percent_interpolation` are worth comparing.
PILLAR_OBJECT = {
    "id": "pillar",
    "pose": [
        1.0, 0.0, 0.0, 0.5,
        0.0, 1.0, 0.0, 0.0,
        0.0, 0.0, 1.0, 0.75,
        0.0, 0.0, 0.0, 1.0,
    ],
    "shape": {"type": "box", "size": [0.2, 0.2, 1.5]},
}

ROBOTS = ["panda", "fanuc", "pr2"]

# Raised off the op's default of 1 so a pair that bullet's manifold reduction
# keeps several contacts for is captured whole. A pair whose contact *count*
# differs is a real disagreement about the manifold, and at 1 it would read as
# agreement on whichever contact each side happened to keep.
MAX_CONTACTS_PER_PAIR = 4


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


def ccd_case(oracle, joint_values, joint_values2):
    result = oracle.ask(
        op="ccd",
        joint_values=joint_values,
        joint_values2=joint_values2,
        objects=[FLOOR_OBJECT, PILLAR_OBJECT],
        max_contacts_per_pair=MAX_CONTACTS_PER_PAIR,
    )
    return {
        "joint_values": joint_values,
        "joint_values2": joint_values2,
        **result,
    }


def capture(robot):
    urdf = REPO_ROOT / "fixtures" / f"{robot}.urdf"
    srdf = REPO_ROOT / "fixtures" / f"{robot}.srdf"
    oracle = Oracle(urdf, srdf)
    try:
        states = oracle.ask(op="random_states", count=RANDOM_CASES, seed=SEED)["states"]
        # Default -> first sampled state, then each consecutive sampled pair.
        pairs = [({}, states[0])]
        pairs += list(zip(states, states[1:]))
        return {"cases": [ccd_case(oracle, a, b) for a, b in pairs]}
    finally:
        oracle.close()


def main():
    FIXTURES_DIR.mkdir(parents=True, exist_ok=True)
    for robot in ROBOTS:
        print(f"capturing {robot}...", file=sys.stderr)
        data = capture(robot)
        out = FIXTURES_DIR / f"{robot}_ccd.json"
        out.write_text(json.dumps(data, indent=2, sort_keys=True) + "\n")
        print(f"wrote {out}", file=sys.stderr)


if __name__ == "__main__":
    main()
