#!/usr/bin/env python3
"""Requirement-anchored message closure -- RUNS INSIDE THE ORACLE CONTAINER.

`measure-requirement-closure.py` stops at the endpoints the C++
`MoveGroupInterface` binds, because the interface *definitions* those
endpoints carry are on neither this host nor the reference checkout:
`moveit_msgs` is not a directory of moveit2, and this host has no `/opt/ros`
at all.  They exist only inside the oracle image, so this half of the
enumeration runs there:

    tag=$(source tools/moveit-oracle/src-digest.sh
          oracle_image_tag "$(oracle_stamp "$PWD/tools/moveit-oracle")")
    sg docker -c "docker run --rm --entrypoint bash \\
        -v $PWD/tools/ci/requirement-message-closure.py:/tmp/rmc.py:ro \\
        $tag -lc 'python3 /tmp/rmc.py'"

The roots are the eight interfaces that client's constructor binds, read off
`move_group_interface.cpp` rather than off the upstream tree -- keep them in
step with `client_endpoints()` in `measure-requirement-closure.py`.  Each is
expanded transitively through its non-primitive field types.

The moveit_msgs half of the result is checked into
`tools/ci/requirement-closure-moveit-msgs.txt` so the port-side half
(`measure-requirement-closure.py --client-messages`) is reproducible without
a container.
"""

import collections
import glob
import os
import re

PRIM = {
    "bool", "byte", "char", "float32", "float64",
    "int8", "uint8", "int16", "uint16", "int32", "uint32", "int64", "uint64",
    "string", "wstring", "time", "duration",
}

ROOTS = [
    ("moveit_msgs", "action", "MoveGroup"),
    ("moveit_msgs", "action", "ExecuteTrajectory"),
    ("moveit_msgs", "srv", "QueryPlannerInterfaces"),
    ("moveit_msgs", "srv", "GetPlannerParams"),
    ("moveit_msgs", "srv", "SetPlannerParams"),
    ("moveit_msgs", "srv", "GetCartesianPath"),
    ("moveit_msgs", "msg", "AttachedCollisionObject"),
    ("std_msgs", "msg", "String"),
]

SHARE = glob.glob("/ws/install/*/share") + glob.glob("/opt/ros/*/share")


def find(pkg: str, kind: str, name: str) -> str | None:
    for s in SHARE:
        for ext in ("msg", "srv", "action"):
            p = os.path.join(s, pkg, kind, f"{name}.{ext}")
            if os.path.exists(p):
                return p
        # some packages keep everything under msg/
        p = os.path.join(s, pkg, "msg", f"{name}.{kind}")
        if os.path.exists(p):
            return p
    return None


def fields(path: str) -> list[str]:
    out = []
    for line in open(path, encoding="utf-8", errors="replace"):
        line = line.split("#")[0].strip()
        if not line or line.startswith("---"):
            continue
        parts = line.split()
        if len(parts) < 2:
            continue
        if "=" in line and len(parts) >= 3 and parts[2] == "=":
            continue  # constant, not a field
        out.append(re.sub(r"\[.*?\]$", "", parts[0]))  # strip array suffix
    return out


def main() -> int:
    if not SHARE:
        print("FAIL: no ROS share/ tree found -- this script must run inside "
              "the oracle image, not on the host")
        return 1

    seen: set[tuple[str, str]] = set()
    missing: list[str] = []
    frontier = list(ROOTS)
    while frontier:
        pkg, kind, name = frontier.pop()
        if (pkg, name) in seen:
            continue
        seen.add((pkg, name))
        path = find(pkg, kind, name)
        if path is None:
            missing.append(f"{pkg}/{name}")
            continue
        for t in fields(path):
            if t in PRIM:
                continue
            p2, n2 = t.split("/", 1) if "/" in t else (pkg, t)
            if (p2, n2) not in seen:
                frontier.append((p2, "msg", n2))

    by_pkg = collections.Counter(p for p, _ in seen)
    print(f"requirement closure from the {len(ROOTS)} endpoints "
          f"MoveGroupInterface binds: {len(seen)} interface types")
    for p, n in by_pkg.most_common():
        print(f"  {n:4d}  {p}")
    print("\n--- moveit_msgs types in the closure ---")
    for p, n in sorted(seen):
        if p == "moveit_msgs":
            print(n)
    if missing:
        print("\n--- unresolved (definition not found in this image) ---")
        for m in sorted(set(missing)):
            print(" ", m)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
