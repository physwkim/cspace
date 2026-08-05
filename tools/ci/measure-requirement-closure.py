#!/usr/bin/env python3
"""Enumerate what the Phase completion conditions REQUIRE, not what the port has.

`measure-port-coverage.py` answers "of the upstream files this project decided
to consider, which are ported".  Its corpus is a *decision*, and a Phase row
can be blocked by something the decision never enumerated -- a whole package,
a message type, an endpoint nobody registered.  Such a thing produces no row
there and no row in `doc/port-coverage.md`, so no existing instrument emits a
signal for it.

This script starts from the other end.  For each row of PORTING-PLAN.md 5's
완료 조건 현황표 it asks: what is this row measured *against*?  Three answers
occur, and each has a different requirement set:

  R-ORACLE   the row compares the port to `tools/moveit-oracle`.  The
             requirement is everything that binary must link -- enumerated
             here as the transitive `#include` closure of its own sources.
  R-CLIENT   the row requires an existing C++ client to interoperate
             unchanged.  The requirement is the set of endpoints that client
             binds, and the message closure of those endpoints.
  R-PORT     the row requires only port-side work (a harness).  No external
             requirement.

The point of the split is that R-ORACLE requirements are almost all inside the
corpus (the corpus was drawn to cover them), while the R-CLIENT requirement is
almost entirely outside it -- which is why a corpus-anchored instrument reports
no gap for a row that is blocked.

What this proves and what it does not:

  * The oracle closure is resolved against the reference checkout by include
    key (path after `/include/`), falling back to a same-directory lookup for
    bare relative includes.  A header reached only through a macro or a
    build-generated path is NOT resolved and lands in the "external" bucket;
    the bucket is printed so that mistake is visible rather than silent.
  * The client endpoint scan is a regex over the client's own translation
    unit.  It finds what the constructor binds.  A lazily-created client in a
    branch it does not scan would be missed, so the count is a lower bound and
    is reported as one.
  * The message closure of those endpoints is NOT computed here, because it
    cannot be: `moveit_msgs` is in neither the reference checkout nor any host
    ROS install.  The definitions exist only inside the oracle container image
    (`/ws/src/moveit_msgs`), so that half of the enumeration is run there --
    the command and its result are recorded in PORTING-PLAN.md.

Usage:
    tools/ci/measure-requirement-closure.py [--upstream DIR] [--repo DIR]
                                            [--list-external] [--check]
"""

from __future__ import annotations

import argparse
import collections
import importlib.util
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_UPSTREAM = "/home/stevek/work/moveit2"

# Every row of 5's 완료 조건 현황표, with the reference it is measured
# against.  `--check` re-reads the table and fails if the two disagree, so
# adding a Phase row without classifying it here is caught rather than
# silently left out of the requirement enumeration.
ROW_REFERENCE = {
    "Phase 0|오라클 FK 1,000세트": "R-ORACLE",
    "Phase 1|panda/prbt/fanuc 링크": "R-ORACLE",
    "Phase 2|FK 10,000": "R-ORACLE",
    "Phase 2|야코비안": "R-ORACLE",
    "Phase 2|관절 한계 클램핑": "R-ORACLE",
    "Phase 3|`collision: bool`": "R-ORACLE",
    "Phase 3|`distance: f64`": "R-ORACLE",
    "Phase 4|(a) 성공률": "R-ORACLE",
    "Phase 4|(b) 성공한 해": "R-ORACLE",
    "Phase 5|제약 조합": "R-ORACLE",
    "Phase 5|제약 샘플러": "R-PORT",
    "Phase 5|씬 diff": "R-ORACLE",
    "Phase 6|TOTG": "R-ORACLE",
    "Phase 7|벤치마크 500건": "R-ORACLE",
    "Phase 7|산출 경로 100%": "R-PORT",
    "Phase 7|경로 길이 중앙값": "R-ORACLE",
    "Phase 8|pilz LIN/PTP/CIRC": "R-ORACLE",
    "Phase 8|CHOMP/STOMP": "R-PORT",
    "Phase 9|기존 C++ `MoveGroupInterface`": "R-CLIENT",
}

ORACLE_SRC = "tools/moveit-oracle/src"

# The translation unit the Phase 9 row names.  Read from the reference
# checkout -- the row says "기존 C++ MoveGroupInterface 클라이언트", so the
# requirement is whatever that client binds, not whatever the plan text lists.
CLIENT_TU = "moveit_ros/planning_interface/move_group_interface/src/move_group_interface.cpp"

INCLUDE = re.compile(r'^\s*#\s*include\s*[<"]([^>"]+)[>"]', re.M)
BIND = re.compile(
    r"(rclcpp_action::)?create_(client|publisher|subscription)"
    r"<\s*([A-Za-z_][A-Za-z0-9_:]*)\s*>",
)
STDLIB = re.compile(r"^[a-z_]+$")


def load_corpus(upstream: str) -> set[str]:
    """The corpus, straight from measure-port-coverage.py -- never redefined here."""
    spec = importlib.util.spec_from_file_location(
        "measure_port_coverage", os.path.join(HERE, "measure-port-coverage.py")
    )
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return set(mod.corpus_files(upstream))


def header_index(upstream: str) -> dict[str, str]:
    out: dict[str, str] = {}
    for dirpath, dirnames, filenames in os.walk(upstream):
        if ".git" in dirnames:
            dirnames.remove(".git")
        for fn in filenames:
            if not fn.endswith((".hpp", ".h")):
                continue
            rel = os.path.relpath(os.path.join(dirpath, fn), upstream)
            i = rel.find("/include/")
            if i >= 0:
                out.setdefault(rel[i + len("/include/") :], rel)
    return out


def oracle_closure(upstream: str, repo: str) -> tuple[set[str], set[str]]:
    """Transitive upstream headers the oracle needs, and the names that are not upstream."""
    index = header_index(upstream)
    seeds: list[tuple[str, str | None]] = []
    src = os.path.join(repo, ORACLE_SRC)
    for fn in sorted(os.listdir(src)):
        if not fn.endswith((".cpp", ".hpp")):
            continue
        with open(os.path.join(src, fn), encoding="utf-8", errors="replace") as fh:
            seeds += [(k, None) for k in INCLUDE.findall(fh.read())]

    def resolve(key: str, src_path: str | None) -> str | None:
        if key in index:
            return index[key]
        if "/" not in key and src_path:
            cand = os.path.join(os.path.dirname(src_path), key)
            if os.path.exists(os.path.join(upstream, cand)):
                return cand
        return None

    seen: set[str] = set()
    resolved: set[str] = set()
    external: set[str] = set()
    frontier = list(seeds)
    while frontier:
        key, src_path = frontier.pop()
        path = resolve(key, src_path)
        tag = path or key
        if tag in seen:
            continue
        seen.add(tag)
        if path is None:
            external.add(key)
            continue
        resolved.add(path)
        with open(os.path.join(upstream, path), encoding="utf-8", errors="replace") as fh:
            frontier += [(k, path) for k in INCLUDE.findall(fh.read())]
    return resolved, external


def client_endpoints(upstream: str) -> list[tuple[str, str]]:
    path = os.path.join(upstream, CLIENT_TU)
    with open(path, encoding="utf-8", errors="replace") as fh:
        found = BIND.findall(fh.read())
    return sorted({("action client" if act else kind, ty) for act, kind, ty in found})


def port_endpoints(repo: str) -> set[str]:
    """Endpoint *names* the port opens, from create_service/publisher/subscription."""
    out: set[str] = set()
    pat = re.compile(r'create_(?:service|publisher|subscription)::<[^>]+>\(\s*"([^"]+)"')
    for dirpath, dirnames, filenames in os.walk(os.path.join(repo, "ros")):
        if "target" in dirnames:
            dirnames.remove("target")
        for fn in filenames:
            if not fn.endswith(".rs"):
                continue
            with open(os.path.join(dirpath, fn), encoding="utf-8", errors="replace") as fh:
                out |= set(pat.findall(fh.read()))
    return out


def check_rows(repo: str) -> int:
    """Fail if 5's table has a row this script has not classified."""
    with open(os.path.join(repo, "PORTING-PLAN.md"), encoding="utf-8") as fh:
        rows = re.findall(r"^\| (Phase \d) \| ([^|]+) \|", fh.read(), re.M)
    unmatched = []
    for phase, cond in rows:
        cond = cond.strip()
        if not any(k.startswith(f"{phase}|") and cond.startswith(k.split("|", 1)[1])
                   for k in ROW_REFERENCE):
            unmatched.append(f"{phase} | {cond}")
    for u in unmatched:
        print(f"UNCLASSIFIED ROW  {u}", file=sys.stderr)
    if unmatched:
        print(
            f"FAIL: {len(unmatched)} of {len(rows)} status rows have no reference "
            f"classification in ROW_REFERENCE",
            file=sys.stderr,
        )
        return 1
    print(f"OK: all {len(rows)} status rows classified ({collections.Counter(ROW_REFERENCE.values())})")
    return 0


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--upstream", default=DEFAULT_UPSTREAM)
    ap.add_argument("--repo", default=os.path.dirname(os.path.dirname(HERE)))
    ap.add_argument("--list-external", action="store_true")
    ap.add_argument("--check", action="store_true")
    args = ap.parse_args()

    if args.check:
        return check_rows(args.repo)

    corpus = load_corpus(args.upstream)
    resolved, external = oracle_closure(args.upstream, args.repo)
    inside = sorted(p for p in resolved if p in corpus)
    outside = sorted(p for p in resolved if p not in corpus)

    print(f"corpus (measure-port-coverage.py, unchanged)  {len(corpus)}")
    print()
    print(f"R-ORACLE requirement closure                  {len(resolved)}")
    print(f"  inside the corpus                           {len(inside)}")
    print(f"  outside the corpus                          {len(outside)}")
    for p in outside:
        print(f"      {p}")

    pkgs = collections.Counter(
        k.split("/")[0] for k in external if "/" in k
    )
    bare = sorted(k for k in external if "/" not in k and not STDLIB.match(k))
    print()
    print(f"names not resolvable in the reference checkout {len(external)}")
    print(f"  external packages                            {len(pkgs)}")
    if args.list_external:
        for k, v in pkgs.most_common():
            print(f"      {v:4d}  {k}")
        if bare:
            print(f"  bare names (generated or oracle-local)       {len(bare)}")
            for k in bare:
                print(f"      {k}")

    print()
    binds = client_endpoints(args.upstream)
    served = port_endpoints(args.repo)
    print(f"R-CLIENT: endpoints {os.path.basename(CLIENT_TU)} binds (lower bound) {len(binds)}")
    for kind, ty in binds:
        print(f"      {kind:20s} {ty}")
    print(f"  endpoint names the port opens               {len(served)}")
    for s in sorted(served):
        print(f"      {s}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
