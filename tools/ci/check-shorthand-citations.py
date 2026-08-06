#!/usr/bin/env python3
"""Freeze the count of shorthand `:NNN` citations so it cannot grow.

A shorthand citation is a backticked `` `:NNN` `` (or `:NNN-MMM`, or a comma
list) whose file is named somewhere earlier in the prose rather than in the
span itself. `check-citation-drift.py`'s corpus is backticked
`path.rs:NNN` spans, so the shorthand form is in NO gate's corpus: not
bounds, not anchor, not content. There are 1,998 of them, and that is where
`6c51a55`'s +3-line shift hid -- of the 55 citations it broke, the drift
gate reported 2.

WHY THIS GATE COUNTS INSTEAD OF RESOLVING. The obvious fix is to resolve
each shorthand to its governing path and bounds-check it. Three rules were
tried and each was refuted by a real site in this tree:

  1. "nearest preceding full path in the enclosing section" -- refuted by
     `PORTING-PLAN.md`'s `EXPECT_TIME_LT` paragraph, where the seven
     shorthands cite an upstream C++ test and the nearest path in section is
     `tools/ci/verify-fixture-provenance.sh`; and by the citation-repair
     tables, whose FIRST column is the citing location, not the cited file.
  2. "the single `.rs` path in the enclosing paragraph" -- refuted by
     `doc/upstream-bugs.md`'s `robot-state-to-stream-group-lookup-unchecked`,
     where `:540`/`:542`/`:548` are governed by
     `moveit_core/robot_state/src/conversions.cpp:535` (upstream, not in this
     tree) and the only `.rs` path in the paragraph is the port's
     `conversions.rs`, a file a third the length.
  3. "nearest preceding full citation of any extension, in reading order" --
     refuted by `ros/moveit-ros/doc/message-mapping.md`, whose bullets
     alternate coordinate systems inside a single sentence:
     ``scene/collision_object.rs:420 (...) -- planning_scene.cpp:1894: ...
     `:1933`: ...``. The `:1933` belongs to the upstream file two clauses
     back, while reading order hands it the port file.

The governing path is a discourse fact, not a lexical one, and each rule
above resolves confidently to the WRONG file rather than failing to resolve
-- which is worse than not checking, because a wrong-but-in-bounds
resolution reads as verified. So this gate does not guess. It bounds the
liability instead: the count per document is frozen, and a document that
grows a new shorthand fails. New citations get written in the full
`path.rs:NNN` form, which `check-citation-drift.py` already checks.

WHAT THE SIBLING GATE RESOLVES ANYWAY, AND WHY THAT IS NOT A CONTRADICTION.
`measure-upstream-citations.py` does resolve some shorthands -- 125 of the
ones counted here, plus 57 more in `.rs` files this gate never scans. That
looked like the two gates holding opposite positions on one question. It is
not; the domains differ, and §NEW measured the boundary rather than arguing
it. That gate inherits only within a SINGLE DOCUMENT LINE, only from an
upstream file, and (since §NEW) never across a switch to another file's line
numbering. Refutation 3 above is out of its reach entirely: the `:1933` and
the `collision_object.rs:420` that mislead reading order sit on two
different source lines of `message-mapping.md`, and that gate drops its base
at every newline, so it never claims that citation at all.

Where the domains DO touch, the same verdict came back. §NEW opened all 469
of that gate's inherited citations: 287 stood on a line that had already
named another coordinate system, and 17 of those meant a file the
inheritance did not give them -- 16 the port `.rs` file, one a `.srdf`
fixture in neither candidate. Every one passed, because the file it was
wrongly handed was long enough to hold the line number. All 287 were
rewritten with their path spelled out and that gate now hard-fails a bare
`:NNN` after a switch, so the residue it still resolves is exactly the
lines where one file is named and one file is meant.

The two budgets interlock rather than compete: converting a shorthand
removes it from this count and, if it names an upstream file, adds it to
that gate's corpus under a key its own baseline freezes. §NEW's conversion
moved 270 across that line in one step, which is what this budget's
shrink-is-also-a-failure rule exists to make visible.

The budget is exact in BOTH directions. Converting a shorthand to a full
path is the point of this gate, and it fails too -- because a budget that
only tracks the ceiling lets a document drop to 150 and silently grow back
to 200. Re-freeze with:

    tools/ci/check-shorthand-citations.py --write-budget

Named `check-` so `.github/workflows/ci.yml`'s glob runs it. Needs nothing
but python3 and the tracked files -- no docker, no cargo, no upstream.
"""

import pathlib
import re
import subprocess
import sys

BUDGET_PATH = pathlib.Path("doc/shorthand-citation-budget.txt")
SHORTHAND_RE = re.compile(r"`:(\d+)(?:-\d+)?(?:,\d+(?:-\d+)?)*`")
# A real fence toggle is 3+ backticks and nothing else with a backtick on the
# line -- these documents also write ```` ```text ```` as an INLINE span while
# discussing fences, and a naive startswith("```") reads those as toggles and
# desyncs for the rest of the file.
FENCE_RE = re.compile(r"^ {0,3}`{3,}[^`]*$")
BUDGET_ROW_RE = re.compile(r"^(\d+)\t(.+)$")

HEADER = """\
# Frozen count of shorthand `:NNN` citations per document, one row per
# document that has any, as "<count>\\ttab<path>". Written by
# tools/ci/check-shorthand-citations.py --write-budget; that script's header
# says why these are counted rather than resolved.
#
# The count is exact in both directions. Growing it means a new citation was
# written in a form no gate can check -- write `path.rs:NNN` instead.
# Shrinking it means a shorthand was converted, which is the goal: re-freeze
# so the lower number becomes the new ceiling.
"""


def tracked_markdown():
    out = subprocess.run(
        ["git", "ls-files", "--deduplicate", "*.md"],
        capture_output=True,
        text=True,
        check=True,
    ).stdout
    return [p for p in out.split("\n") if p]


def count_shorthand(path):
    """Shorthand spans outside every fenced block."""
    try:
        lines = pathlib.Path(path).read_text(encoding="utf-8").split("\n")
    except (OSError, UnicodeDecodeError) as exc:
        raise SystemExit(f"FAIL {path}: unreadable ({exc})")
    in_fence = False
    total = 0
    for line in lines:
        if FENCE_RE.match(line):
            in_fence = not in_fence
            continue
        if in_fence:
            continue
        total += len(SHORTHAND_RE.findall(line))
    return total


def measure():
    counts = {}
    for path in tracked_markdown():
        n = count_shorthand(path)
        if n:
            counts[path] = n
    return counts


def parse_budget(text):
    budget = {}
    for lineno, line in enumerate(text.split("\n"), 1):
        if not line.strip() or line.lstrip().startswith("#"):
            continue
        match = BUDGET_ROW_RE.match(line)
        if match is None:
            raise SystemExit(
                f"FAIL {BUDGET_PATH}:{lineno}: row is not '<count>\\t<path>': {line!r}"
            )
        budget[match.group(2)] = int(match.group(1))
    return budget


def render(counts):
    rows = "".join(f"{n}\t{p}\n" for p, n in sorted(counts.items()))
    return HEADER + rows


def main():
    counts = measure()

    if "--write-budget" in sys.argv:
        BUDGET_PATH.parent.mkdir(parents=True, exist_ok=True)
        BUDGET_PATH.write_text(render(counts), encoding="utf-8")
        print(
            f"wrote {BUDGET_PATH}: {len(counts)} document(s), "
            f"{sum(counts.values())} shorthand citation(s)"
        )
        return 0

    if not BUDGET_PATH.is_file():
        print(
            f"FAIL {BUDGET_PATH} does not exist. Create it with "
            f"`{sys.argv[0]} --write-budget`.",
            file=sys.stderr,
        )
        return 1

    budget = parse_budget(BUDGET_PATH.read_text(encoding="utf-8"))

    # A zero parse and a clean tree are otherwise the same exit code.
    if not counts:
        print(
            "FAIL counted zero shorthand citations across every tracked .md -- "
            "the corpus grammar changed and this checked nothing",
            file=sys.stderr,
        )
        return 1

    failures = []
    for path in sorted(set(counts) | set(budget)):
        now = counts.get(path, 0)
        was = budget.get(path)
        if was is None:
            failures.append(
                f"{path}: {now} shorthand citation(s) and no budget row. A new "
                f"document may not introduce them -- write `path.rs:NNN`, which "
                f"check-citation-drift.py checks."
            )
        elif now > was:
            failures.append(
                f"{path}: {now} shorthand citation(s), budget {was} (+{now - was}). "
                f"A shorthand `:NNN` is in no gate's corpus -- write the new "
                f"citation as `path.rs:NNN` instead."
            )
        elif now < was:
            failures.append(
                f"{path}: {now} shorthand citation(s), budget {was} ({now - was}). "
                f"Converting them is the goal -- re-freeze so the lower number "
                f"becomes the ceiling: `{sys.argv[0]} --write-budget`."
            )

    print(
        f"{sum(counts.values())} shorthand citation(s) across {len(counts)} "
        f"document(s); budget covers {len(budget)}."
    )

    if failures:
        for failure in failures:
            print("FAIL " + failure, file=sys.stderr)
        return 1

    print("OK no document grew a citation form that nothing checks")
    return 0


if __name__ == "__main__":
    sys.exit(main())
