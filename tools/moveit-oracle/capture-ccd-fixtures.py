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

Every pair is captured twice more with a non-convex mesh attached at the arm's
tip (see `MESH_VERTICES`): that is the one scene in which `CollisionEnvBullet`
asks `createShapePrimitive` for `USE_SHAPE_TYPE` on a mesh, and so the only one
that sweeps a compound of `btTriangleShapeEx` rather than a convex hull.

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

# The link each robot's mesh body is attached to -- the last link of the arm,
# so the sweep between two sampled states carries the body across the pillar
# instead of pivoting it near the base.
ATTACH_LINK = {"panda": "panda_hand", "fanuc": "tool0", "pr2": "r_gripper_palm_link"}

# A four-triangle open V: two 0.16x0.10 panels meeting along the y axis at the
# origin and rising to z = 0.08 at both ends.
#
# Non-convex on purpose. `addAttachedObjects` is the one caller that asks
# `createShapePrimitive` for `USE_SHAPE_TYPE` on a mesh
# (`collision_env_bullet.cpp:345-346`), which builds a compound of one
# `btTriangleShapeEx` per face rather than a convex hull -- and against a
# convex mesh the two are the same solid, so a port that took the hull branch
# for an attached body would match this fixture case for case. The V's hull
# fills the valley the soup leaves open, so the pillar entering it separates
# them.
MESH_VERTICES = [
    [-0.08, -0.05, 0.08],
    [0.00, -0.05, 0.00],
    [0.08, -0.05, 0.08],
    [-0.08, 0.05, 0.08],
    [0.00, 0.05, 0.00],
    [0.08, 0.05, 0.08],
]
MESH_TRIANGLES = [[0, 1, 4], [0, 4, 3], [1, 2, 5], [1, 5, 4]]

# The two scenes every state pair is captured in: no attached body, and the
# mesh above attached at the arm's tip with its own frame the link's.
#
# Both, not just the second. An attached body is the *last* object into the
# manager, not the first: links go in at construction
# (`collision_env_bullet.cpp:60-63` -> `addLinkAsCollisionObject` -> `:442`) and
# world objects when the world announces them, while an attached body is added
# during the query at `:216-225` and removed again at `:237`. So the attached
# scene is the unattached one plus a proxy appended after everything else, and
# the pair prefix a tight `max_contacts` keeps is not the prefix the same two
# states produce without it -- which is what the unattached half pins.
def attached_scenes(robot):
    return [
        [],
        [
            {
                "id": "carried_mesh",
                "link_name": ATTACH_LINK[robot],
                "shapes": [
                    {
                        "type": "mesh",
                        "vertices": MESH_VERTICES,
                        "triangles": MESH_TRIANGLES,
                    }
                ],
                "shape_poses": [
                    [
                        1.0, 0.0, 0.0, 0.0,
                        0.0, 1.0, 0.0, 0.0,
                        0.0, 0.0, 1.0, 0.0,
                        0.0, 0.0, 0.0, 1.0,
                    ]
                ],
            }
        ],
    ]


ROBOTS = ["panda", "fanuc", "pr2"]

# The two contact budgets every state pair is captured at.
#
# The loose one raises `max_contacts_per_pair` off the op's default of 1 so a
# pair that bullet's manifold reduction keeps several contacts for is captured
# whole -- at 1, two sides that disagree about the manifold read as agreeing on
# whichever contact each happened to keep -- and leaves `max_contacts` at a
# figure no scene here reaches, so every pair found is stored.
#
# The tight one is what makes insertion order observable at all. Below the
# budget, the result is a `std::map` keyed by the sorted name pair and the
# order the objects entered the manager cannot be read off it; a port that
# added its links in a different order agrees case for case. Spend the budget
# and `processResult` keeps whichever contacts arrived first, which is the
# order `createProxy` announced the overlaps in. 3 rather than 1 so the cut
# lands inside the traversal rather than on its first step.
BUDGETS = [
    {"max_contacts": 100, "max_contacts_per_pair": 4},
    {"max_contacts": 3, "max_contacts_per_pair": 1},
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


def ccd_case(oracle, joint_values, joint_values2, budget, attached_bodies):
    result = oracle.ask(
        op="ccd",
        joint_values=joint_values,
        joint_values2=joint_values2,
        objects=[FLOOR_OBJECT, PILLAR_OBJECT],
        attached_bodies=attached_bodies,
        **budget,
    )
    # `nearest_points` is dropped rather than stored. `addCastSingleResult`
    # never assigns it and `collision_detection::Contact` gives it no
    # initialiser, so every swept contact carries whatever the stack held --
    # values like `2.07e-312` and `-1.60e+268` that differ between two runs of
    # this script over the same states. Storing them would make a re-capture
    # print a diff on every case whether or not anything about the geometry
    # moved, which is exactly the signal these fixtures exist to carry.
    for contact in result.get("contacts", []):
        contact.pop("nearest_points", None)

    # The budget travels with the case rather than being a constant the reader
    # restates: a case answered at one budget and replayed at another compares
    # two different questions, and nothing in the response says which was
    # asked.
    return {
        "joint_values": joint_values,
        "joint_values2": joint_values2,
        "attached_bodies": attached_bodies,
        **budget,
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
        return {
            "cases": [
                ccd_case(oracle, a, b, budget, attached)
                for attached in attached_scenes(robot)
                for budget in BUDGETS
                for a, b in pairs
            ]
        }
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
