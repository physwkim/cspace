#!/usr/bin/env python3
"""Independent re-derivation of corpus / ported / unported.

Deliberately shares NO code with measure-port-coverage.py.  The rules are
transcribed from doc/port-coverage.md §1 (the prose definition), and the
"ported" test is built the other way round: instead of parsing the citation
grammar into a set of paths, each citation BLOCK is captured as raw text and
each corpus file is asked whether some block names it.

Set comparison, not total comparison -- a matching total can hide two row
errors that cancel.

This lived in a scratch directory for two rounds while §258 and §261 quoted
its output, which is a published number resting on a file no checkout has.
It is here so those two sections can be re-run.  Its numbers must keep
agreeing with measure-port-coverage.py's; where they disagree, the
disagreement is the finding, and neither figure may be published alone.

    tools/ci/measure-port-coverage-independent.py [REPO] [--list-unported]
                                                         [--list-ported]
"""
from __future__ import annotations

import os
import re
import sys

UP = "/home/stevek/work/moveit2"
# A leading-dash argument is a flag, not a path.  Taking `sys.argv[1]`
# unconditionally made `--list-unported` the repo root, which walks nothing,
# names nothing, and prints `ported 0 / unported 245` rather than an error --
# a verification instrument that fails toward a plausible wrong answer.
_pos = [a for a in sys.argv[1:] if not a.startswith("-")]
REPO = _pos[0] if _pos else os.getcwd()

ROOTS = ["moveit_core", "moveit_kinematics", "moveit_planners/chomp",
         "moveit_planners/stomp", "moveit_planners/pilz_industrial_motion_planner"]
EXCL_CORE = {"controller_manager", "collision_detection_bullet",
             "collision_detection_fcl", "version"}
SHIM = ".h header is obsolete. Please use the .hpp header instead."


def corpus() -> list[str]:
    out = []
    for root in ROOTS:
        base = os.path.join(UP, root)
        for dp, dn, fns in os.walk(base):
            rel = os.path.relpath(dp, UP)
            parts = rel.split(os.sep)
            if "test" in parts or "tests" in parts:
                dn[:] = []
                continue
            if parts[0] == "moveit_core" and len(parts) > 1 and parts[1] in EXCL_CORE:
                dn[:] = []
                continue
            for fn in fns:
                if not fn.endswith((".cpp", ".hpp", ".h")):
                    continue
                p = os.path.join(dp, fn)
                if fn.endswith(".h"):
                    with open(p, encoding="utf-8", errors="replace") as fh:
                        if SHIM in fh.read():
                            continue
                out.append(os.path.relpath(p, UP))
    return sorted(out)


HDR = re.compile(r"^\s*//\s*Ported from moveit2 @ [0-9a-f]{40}:\s*$")


def blocks() -> list[tuple[str, list[str]]]:
    """(rs_path, [raw comment body lines]) for every citation block."""
    out = []
    for top in ("crates", "ros"):
        for dp, dn, fns in os.walk(os.path.join(REPO, top)):
            if "target" in dp.split(os.sep):
                continue
            for fn in fns:
                if not fn.endswith(".rs"):
                    continue
                p = os.path.join(dp, fn)
                lines = open(p, encoding="utf-8", errors="replace").readlines()
                i = 0
                while i < len(lines):
                    if not HDR.match(lines[i]):
                        i += 1
                        continue
                    i += 1
                    body = []
                    while i < len(lines):
                        m = re.match(r"^\s*//(.*)$", lines[i])
                        if not m:
                            break
                        b = m.group(1)
                        if not b.strip() or "not ported" in b:
                            break
                        body.append(b)
                        i += 1
                    out.append((os.path.relpath(p, REPO), body))
    return out


def named_by(body: list[str], f: str) -> bool:
    """Does this raw block name upstream path `f`?

    Three shapes, resolved independently of the other instrument's grammar:
      * the full path appears verbatim on some line
      * a `dir/` line appears with NO indented member lines under it
        (whole-directory citation) and f is under that dir
      * a `dir/` line is followed by indented member lines, one of which is
        f's basename
      * a brace form `dir/{a,b}` whose expansion contains f
    """
    d, base = os.path.split(f)
    d += "/"
    for ln in body:
        if f in ln:
            return True
    # brace expansion
    for ln in body:
        m = re.search(r"(moveit_[A-Za-z0-9_./-]*/)\{([^}]*)\}", ln)
        if m and m.group(1) == d and base in [x.strip() for x in m.group(2).split(",")]:
            return True
    # directory citation, with or without members
    for idx, ln in enumerate(body):
        t = ln.strip()
        if not re.fullmatch(r"moveit_[A-Za-z0-9_./-]*/", t):
            continue
        members = []
        j = idx + 1
        while j < len(body):
            nxt = body[j]
            nt = nxt.strip()
            ind = len(nxt) - len(nxt.lstrip())
            if re.fullmatch(r"moveit_[A-Za-z0-9_./-]*/", nt):
                break  # the next directory citation starts here
            if ind > 3:
                # An indented line that is not a source member -- e.g.
                # `cartesian_limits_parameters.yaml` under
                # `pilz_industrial_motion_planner/src/`.  Skipping the block
                # here (rather than continuing) leaves `members` empty and
                # turns a SCOPED directory citation into a whole-directory
                # one: that alone moved 8 pilz `src/*.cpp` files from
                # unported to ported while the corpus total stayed 245.
                if re.fullmatch(r"[A-Za-z0-9_.-]+\.(cpp|hpp|h|cc)(\s.*)?", nt):
                    members.append(nt.split()[0])
                j += 1
                continue
            break
        if members:
            if t == d and base in members:
                return True
        else:
            if f.startswith(t):
                return True
    return False


def main() -> int:
    c = corpus()
    bl = blocks()
    ported, unported = [], []
    for f in c:
        if any(named_by(b, f) for _, b in bl):
            ported.append(f)
        else:
            unported.append(f)
    print(f"corpus   {len(c)}")
    print(f"ported   {len(ported)}")
    print(f"unported {len(unported)}")
    if "--list-unported" in sys.argv:
        print("\n--- unported ---")
        for f in unported:
            print(f)
    if "--list-ported" in sys.argv:
        print("\n--- ported ---")
        for f in ported:
            print(f)
    return 0


if __name__ == "__main__":
    sys.exit(main())
