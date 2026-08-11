#!/usr/bin/env python3
# Copyright (c) 2026, cspace contributors
# SPDX-License-Identifier: BSD-3-Clause
"""Checks that every `<file>.cpp:N-M` citation in the CCD port points at code.

Run through `tools/ci/verify-upstream-citations.sh`, which locates the two
pinned upstream trees and skips loudly when either is absent.

# What a citation is, here

The continuous-collision port documents itself almost entirely by citing
upstream: a doc comment names a C++ symbol and gives the file and line span
its behaviour was read off. Nothing in this workspace resolved those spans
before this gate, and 44 of the 459 in the port were wrong when it was
written -- a blank line, a neighbouring function, `btSetMax` where the
sentence said `btSetMin`. They resolve to a real file at a real line, so a
range check reports them clean; they were found by opening all 459 by hand.

# The two rules, and why only two

Each is decidable from the two texts alone, with no judgment about what a
sentence meant:

  RESOLVE  the cited path suffix names exactly one file in the two trees.
           Neither zero nor several is a claim anything can grade, and a
           skip counter here would report OK on citations it never read --
           it hid two wrong spans while `collision_common.hpp` was
           ambiguous. There is no skip category for that reason.
  RANGE    `1 <= lo <= hi <= len(file)`. A span past EOF cites nothing.
  START    the first cited line is not blank. A span that opens on the
           blank line above its function is off by one, which is how eleven
           of the port's were wrong; and a blank line cannot be what any
           sentence is about.

What is deliberately NOT a rule:

  - The same test on the *last* line. It reads symmetric and is not: a span
    that ends one line late still contains its whole construct, while one
    that starts one line late has dropped the signature that names it.
    Measured before it was rejected -- 20 spans here end on the blank after
    their construct (`btGjkEpa2.cpp:155-553`, `btGjkPairDetector.cpp:78-99`,
    ...), consistently enough to be this port's habit, and no defect found in
    the hand audit sat at an end boundary.
  - A *comment* first line. Citing a function together with the `/** @brief
    ... */` above it, or bullet3's bare `//` separator, is this port's own
    convention (`bullet_utils.hpp:661-669` is the declaration plus the doc
    that states the constraint the sentence is about). 42 citations do it and
    every one read correct, so a rule here would be 42 exemptions and no
    finding.
  - Whether the span *contains* the symbol the sentence names. That is the
    check that would have caught the other 33, and it cannot be mechanised
    from the citing line: the subject is often a line or two above it, and a
    backtick on the citing line as often belongs to the neighbouring clause.
    Tried as a heuristic while this gate was written -- 53 flags, 24 real, 29
    noise. A gate at that precision is an allowlist of 29 judgments, which
    vouches for them rather than checking them.

So this gate closes the mechanically-decidable half and says so. The half it
cannot see was audited by hand once, at the commit that introduced this file;
a citation written after that is checked by these two rules and by review, not
by anything here.

# Scope

`crates/cspace-bullet`, `crates/cspace-bullet-cast`, and the two
continuous-collision files in `cspace-collision`. The rest of the workspace
cites upstream the same way and is NOT checked: seven citations outside this
scope have a blank first line today (see the wrapper's header), and widening
the scope means either fixing files this port did not touch or shipping a
gate that is born red.
"""
import pathlib
import re
import sys
from collections import defaultdict

# One citation, e.g. `btDbvt.h:131-172` or `bullet_utils.cpp:196`. The
# open-ended `:63-...` form this port uses for "from here on" matches as a
# single line, which is what it should be checked as -- only its start is a
# claim about a position.
#
# A citation may carry leading path components, and must where the bare name is
# not unique upstream: three files are called `collision_common.hpp`, so those
# citations give the include path (`moveit/collision_detection/...`) that tells
# them apart. Anything shorter is not a worse citation, it is an unresolvable
# one, and this gate counts it as skipped rather than picking a candidate.
CITE = re.compile(
    r"\b((?:[A-Za-z_][A-Za-z0-9_.-]*/)*[A-Za-z_][A-Za-z0-9_]*\.(?:cpp|hpp|h))\s*:\s*(\d+)(?:-(\d+))?"
)

SUBJECTS = [
    "crates/cspace-bullet/src",
    "crates/cspace-bullet-cast/src",
    "crates/cspace-collision/src/bullet_ccd.rs",
    "crates/cspace-collision/tests/ccd_parity.rs",
]


def main() -> int:
    if len(sys.argv) != 4:
        print(f"usage: {sys.argv[0]} <repo-root> <moveit2-root> <bullet3-root>", file=sys.stderr)
        return 2
    repo, moveit2, bullet3 = (pathlib.Path(a).resolve() for a in sys.argv[1:4])

    for root in (repo, moveit2, bullet3):
        if not root.is_dir():
            print(f"FAIL {root} is not a directory", file=sys.stderr)
            return 2

    # Every path suffix, on component boundaries, of every upstream source:
    # `collision_common.hpp`, `collision_detection/collision_common.hpp`,
    # `moveit/collision_detection/collision_common.hpp`, and so on. A citation
    # resolves when the suffix it gives is held by exactly one file, so a bare
    # name works wherever it is unique and a longer one is available where it
    # is not. Suffixing rather than substring-matching keeps
    # `detection/collision_common.hpp` from matching `collision_detection/...`.
    index = defaultdict(list)
    for root in (moveit2, bullet3):
        for p in root.rglob("*"):
            if p.is_file() and p.suffix in (".cpp", ".hpp", ".h"):
                parts = p.relative_to(root).parts
                for i in range(len(parts)):
                    index["/".join(parts[i:])].append(p)
    if not index:
        print("FAIL indexed no upstream sources -- the roots are not the trees this expects.",
              file=sys.stderr)
        return 2

    files = []
    for s in SUBJECTS:
        p = repo / s
        if not p.exists():
            print(f"FAIL {s} does not exist -- this gate's scope has moved.", file=sys.stderr)
            return 2
        files.extend(sorted(p.rglob("*.rs")) if p.is_dir() else [p])

    def rel_to_tree(p: pathlib.Path) -> str:
        # The candidates a RESOLVE failure lists are only useful as the path
        # the citation should have given more of, so print each from its own
        # tree root rather than from this machine's filesystem root.
        for root in (moveit2, bullet3):
            if p.is_relative_to(root):
                return str(p.relative_to(root))
        return str(p)

    cache: dict[pathlib.Path, list[str]] = {}

    def lines_of(path: pathlib.Path) -> list[str]:
        if path not in cache:
            cache[path] = path.read_text(errors="replace").splitlines()
        return cache[path]

    checked = 0
    failures: list[str] = []
    for f in files:
        rel = f.relative_to(repo)
        for lineno, line in enumerate(f.read_text(errors="replace").splitlines(), 1):
            for name, a, b in CITE.findall(line):
                hits = index.get(name, [])
                lo, hi = int(a), int(b or a)
                where = f"{rel}:{lineno} cites {name}:{a}" + (f"-{b}" if b else "")
                checked += 1
                if not hits:
                    failures.append(f"RESOLVE {where} -- no such file in either tree")
                    continue
                if len(hits) > 1:
                    names = ", ".join(sorted(rel_to_tree(h) for h in hits))
                    failures.append(
                        f"RESOLVE {where} -- names {len(hits)} files, "
                        f"give enough of the path to tell them apart: {names}"
                    )
                    continue
                up = lines_of(hits[0])
                if lo < 1 or hi < lo or hi > len(up):
                    failures.append(f"RANGE {where} -- that file has {len(up)} lines")
                    continue
                if not up[lo - 1].strip():
                    failures.append(f"START {where} -- line {lo} is blank")

    print(f"{checked} citation(s) checked against the pinned upstream trees "
          f"in {len(files)} file(s)")

    if not checked:
        print("FAIL checked no citation -- this gate would report OK having examined nothing.",
              file=sys.stderr)
        return 1

    if failures:
        print()
        for line in failures:
            print(f"FAIL {line}", file=sys.stderr)
        print(f"\nFAIL {len(failures)} citation(s) do not point at code.", file=sys.stderr)
        return 1

    print("ok   every citation resolves to one file, is in range, and opens on a non-blank line")
    return 0


if __name__ == "__main__":
    sys.exit(main())
