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
before this gate, and 46 of the 465 in the port were wrong when it was
written -- a blank line, a neighbouring function, `btSetMax` where the
sentence said `btSetMin`. They resolve to a real file at a real line, so a
range check reports them clean; they were found by opening all 465 by hand.

# The rules, and why only these

Each is decidable from the two texts alone, with no judgment about what a
sentence meant:

  RESOLVE  the cited path suffix names exactly one file in the two trees.
           Neither zero nor several is a claim anything can grade, and a
           skip counter here would report OK on citations it never read --
           it hid two wrong spans while `collision_common.hpp` was
           ambiguous. There is no skip category for that reason.
  RANGE    `1 <= lo <= hi <= len(file)`. A span past EOF cites nothing.
  START    the first cited line is not blank. A span that opens on the
           blank line above its function is off by one, which is how 26 of
           the port's were wrong; and a blank line cannot be what any
           sentence is about.
  SUBJECT  a citation written without a filename -- `` `:227-231` `` -- means
           the file this module declares, and a module that writes one must
           declare it. RANGE and START then apply to it exactly as to a
           written-out citation.
  SPLIT    no comment line ends at a citation's colon. `btTriangleShape.h:`
           wrapped onto `60-64` is a citation none of the rules above can
           see, so it read as clean while pointing nowhere checkable.

# The unqualified form, and why it needed a declaration rather than a reading

160 of the port's citations gave a line and no file. Nothing resolved them,
and the three obvious readings all fail on the text as written:

  - *the nearest preceding filename.* 19 flags, and the first three opened by
    hand were false -- `convex_convex.rs`'s `:411-697`, `:704-775`, `:785-788`
    all mean `btConvexConvexAlgorithm.cpp`, while the nearest citation above
    them is `bullet_utils.cpp`.
  - *the module's own `Ported from` header.* `dbvt.rs` ports `btDbvt.cpp` and
    its one unqualified citation is about MoveIt's call site;
    `contact_test_data.rs` ports `contact_checker_common.cpp` and cites
    `bullet_utils.hpp`.
  - *whichever candidate the line number fits.* 143 of the 160 stayed
    ambiguous: a 2700-line upstream file makes almost any span "fit".

So the file states its subject and the gate reads that statement, rather than
inferring one. 13 modules declare theirs; the other four -- `shapes.rs` cites
8 upstream files across 13 citations, `manifold.rs` 3 across 3 -- have no
subject to declare, and there every citation is written out. Writing the
filename out is always allowed and always wins; the declaration only says what
the short form abbreviates.

Reading the same 160 against the declarations found 13 wrong (`fd45b960`,
`ee7c403a`) -- 7 opening on the blank line above their construct, 6 opening on
code that is not what the sentence names.

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
    check that would have caught the other 20, and it cannot be mechanised
    from the citing line: the subject is often a line or two above it, and a
    backtick on the citing line as often belongs to the neighbouring clause.
    Tried as a heuristic while this gate was written -- 53 flags, 24 real, 29
    noise. A gate at that precision is an allowlist of 29 judgments, which
    vouches for them rather than checking them.

So this gate closes the mechanically-decidable half and says so. The half it
cannot see was audited by hand once, at the commit that introduced this file;
a citation written after that is checked by the rules above and by review, not
by anything here.

# Scope, and what running wider would report today

`crates/cspace-bullet`, `crates/cspace-bullet-cast`, and the two
continuous-collision files in `cspace-collision` -- 500 written-out citations
plus 127 unqualified ones, out of the workspace's 1733 and 877. What the rest
would report is measured, not assumed: these same rules over every tracked
`.rs`, against moveit2 and all eight `third_party/` trees, give 0 RANGE, 0
START, 0 SPLIT, and 416 RESOLVE.

SUBJECT cannot be measured that way, because outside this scope no module
declares one: 750 unqualified citations sit in 88 files there, and assigning
each file a subject is the same per-file audit the 17 modules here took.
Those 750 are unchecked, by this gate and by anything else.

RANGE, START and SPLIT are zero because the sites that were not have since
been fixed (`b20c3c86`, `74401d9d`, and the SPLIT one in `cspace-core`'s
`shapes.rs` with the two in `moveit-diff`). The gate stays scoped to the port
because of the 416, and two things are worth knowing before anyone widens it:

  - 136 of them resolve to nothing, 280 to several files. Both counts are
    dominated by the two shapes already found defective inside the port: a
    path that skips a directory (`collision_detection_fcl/
    collision_common.cpp`, 27 citations, fixed in `e0ca35ea`, where upstream
    has `src/` between those two components) and a bare basename several
    packages share (`planning_scene.cpp`, 110 citations, matching
    moveit_core's and moveit_py's).
  - So the 416 is not a clean "cites an upstream we do not vendor" residue.
    It is a mixture, and separating the two is per-citation work of the same
    kind that fixed the 28 -- not something this gate can widen into by being
    handed more trees.
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
# The stem takes `-` and `.` as well as word characters, or the leading `\b`
# lets a longer name match its own tail: `gjk_solver_libccd-inl.h:679` was read
# as `inl.h:679`, which resolves to nothing today and would resolve to the
# wrong file the day an `inl.h` is vendored. 95 citations outside this gate's
# scope are written that way.
CITE = re.compile(
    r"\b((?:[A-Za-z_][A-Za-z0-9_.-]*/)*[A-Za-z_][A-Za-z0-9_.-]*\.(?:cpp|hpp|h))\s*:\s*(\d+)(?:-(\d+))?"
)

# The same citation with the filename left off, which is how 160 of the port's
# are written. Backticks on both sides are what separates it from prose: a bare
# `:5` in running text is a colon and a number, and the port has plenty.
BARE = re.compile(r"`:(\d+)(?:-(\d+))?`")

# A module's statement of what its unqualified citations abbreviate. Matched
# against the module doc with its `//!` markers stripped and its lines joined,
# so the sentence may wrap anywhere -- as it does in 5 of the 13 modules. The
# stem carries no backtick of its own, or the capture would start at whichever
# inline code span came first.
DECLARES = re.compile(r"Unqualified citations in this file are lines in\s+`([^`]+)`")

# A citation whose line number wrapped onto the next comment line. CITE cannot
# match across the break, so the citation is invisible to every rule above and
# reports as nothing to check -- `shapes.rs` carried one for the life of the
# port, and three more sit outside this gate's scope.
SPLIT = re.compile(r"[A-Za-z_][A-Za-z0-9_.-]*\.(?:cpp|hpp|h)\s*:\s*$")
COMMENT = re.compile(r"^\s*(?://|\*)")

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

    failures: list[str] = []

    def declared_subject(src: list[str], rel: pathlib.PurePath) -> str | None:
        """The file the module says its unqualified citations abbreviate.

        Checked here rather than at its first use, or a declaration that
        stopped resolving -- the file renamed upstream, the module's short
        form no longer unique -- would sit unread for as long as nobody
        happened to write an unqualified citation under it.
        """
        doc = " ".join(ln[3:].strip() for ln in src if ln.startswith("//!"))
        found = DECLARES.findall(doc)
        if not found:
            return None
        if len(found) > 1:
            failures.append(
                f"SUBJECT {rel} declares {len(found)} subjects ({', '.join(found)}) -- "
                f"an unqualified citation would have no single reading"
            )
            return None
        hits = index.get(found[0], [])
        if len(hits) != 1:
            failures.append(
                f"SUBJECT {rel} declares {found[0]}, which names {len(hits)} files "
                f"in the two trees"
            )
            return None
        return found[0]

    checked = 0
    bare = 0
    for f in files:
        rel = f.relative_to(repo)
        src = f.read_text(errors="replace").splitlines()
        subject = declared_subject(src, rel)

        def check(name: str, a: str, b: str, where: str) -> None:
            nonlocal checked
            checked += 1
            hits = index.get(name, [])
            lo, hi = int(a), int(b or a)
            if not hits:
                failures.append(f"RESOLVE {where} -- no such file in either tree")
                return
            if len(hits) > 1:
                names = ", ".join(sorted(rel_to_tree(h) for h in hits))
                failures.append(
                    f"RESOLVE {where} -- names {len(hits)} files, "
                    f"give enough of the path to tell them apart: {names}"
                )
                return
            up = lines_of(hits[0])
            if lo < 1 or hi < lo or hi > len(up):
                failures.append(f"RANGE {where} -- that file has {len(up)} lines")
                return
            if not up[lo - 1].strip():
                failures.append(f"START {where} -- line {lo} is blank")

        for lineno, line in enumerate(src, 1):
            for name, a, b in CITE.findall(line):
                span = f"{a}-{b}" if b else a
                check(name, a, b, f"{rel}:{lineno} cites {name}:{span}")
            for a, b in BARE.findall(line):
                bare += 1
                span = f"{a}-{b}" if b else a
                if subject is None:
                    failures.append(
                        f"SUBJECT {rel}:{lineno} cites `:{span}` with no filename, and this "
                        f"module declares no subject -- either add the declaration or write "
                        f"the filename out"
                    )
                    continue
                check(subject, a, b, f"{rel}:{lineno} cites `:{span}`, declared {subject}")
            if COMMENT.match(line) and SPLIT.search(line.rstrip()):
                failures.append(
                    f"SPLIT {rel}:{lineno} ends at a citation's colon -- its line number "
                    f"wrapped onto the next line, where no rule here can see it"
                )

    print(f"{checked} citation(s) checked against the pinned upstream trees "
          f"in {len(files)} file(s), {bare} of them written without a filename")

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

    print("ok   every citation names one file -- written out, or through the module's "
          "declared subject -- is in range, and opens on a non-blank line")
    return 0


if __name__ == "__main__":
    sys.exit(main())
