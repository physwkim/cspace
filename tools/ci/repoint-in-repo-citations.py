#!/usr/bin/env python3
"""Re-derive the in-repo `PORTING-PLAN.md:NNN` citations a merge moved.

A merge that inserts lines mid-file shifts every citation target below the
insertion point. `check-citation-drift.py` then reports those targets as
`blank-line` or `section-mismatch`: the citation still resolves, it just
lands on the wrong line now. Repairing them by hand is arithmetic over one
delta per insertion point, and that is where the mistakes come from.

Two rules make this safe to run:

1. **The gate decides what is broken, not this script.** It runs
   `check-citation-drift.py` and repairs exactly the citations that gate
   lists in a failing class. A citation the gate is content with is left
   alone even if its target moved -- re-deriving the corpus here would mean
   two parsers that can disagree, and the gate's is the one that counts.

2. **Relocation is by content, never by arithmetic.** For each stale target
   it takes the lines that stood there in <old-rev> and finds where that
   exact block sits today. A block that is blank, missing, or non-unique is
   NOT rewritten -- it is listed for a human, because content alone cannot
   then date the citation, and a blank target was already wrong before the
   merge.

    tools/ci/repoint-in-repo-citations.py <old-rev> [--apply]

Without --apply it prints the mapping and changes nothing. `!`-sigil
citations are never touched: they name a line and assert nothing about it.

This is a repair tool, not a gate -- deliberately not named `check-*` or
`verify-*`, so the CI glob does not run it.
"""
import re
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
PLAN = "PORTING-PLAN.md"
FAILING_CLASSES = ("blank-line", "section-mismatch", "out-of-bounds", "unresolvable")
CITER_GLOBS = ("*.md", "*.py", "*.sh", "*.json")
# The gate's second-population listing: `    <class>  <citer:line>  `<cite>``
LISTING_RE = re.compile(
    r"^\s+(" + "|".join(FAILING_CLASSES) + r")\s+\S+\s+`" + re.escape(PLAN) + r":(\d+)(?:-(\d+))?`"
)


def fail(msg):
    print(f"FAIL {msg}", file=sys.stderr)
    raise SystemExit(1)


def gate_failures():
    """(a, b) spans the drift gate puts in a failing class."""
    proc = subprocess.run(
        [sys.executable, str(ROOT / "tools/ci/check-citation-drift.py")],
        capture_output=True, text=True)
    spans = set()
    for line in (proc.stdout + proc.stderr).splitlines():
        m = LISTING_RE.match(line)
        if m:
            a = int(m.group(2))
            spans.add((a, int(m.group(3)) if m.group(3) else a))
    return sorted(spans)


def relocate(old_lines, new_lines, a, b):
    if not (1 <= a <= b <= len(old_lines)):
        return None, "out of bounds in <old-rev>"
    block = old_lines[a - 1:b]
    if not any(l.strip() for l in block):
        return None, "target was already blank in <old-rev>"
    span = b - a
    hits = [i for i in range(len(new_lines) - span)
            if new_lines[i:i + span + 1] == block]
    if len(hits) == 1:
        return (hits[0] + 1, hits[0] + 1 + span), None
    return None, ("content not found today" if not hits else f"{len(hits)} matches, not unique")


def main():
    if len(sys.argv) < 2:
        fail(f"usage: {sys.argv[0]} <old-rev> [--apply]")
    old_rev, apply = sys.argv[1], "--apply" in sys.argv[2:]

    old_lines = subprocess.run(
        ["git", "-C", str(ROOT), "show", f"{old_rev}:{PLAN}"],
        capture_output=True, text=True, check=True).stdout.splitlines()
    new_lines = (ROOT / PLAN).read_text(encoding="utf-8").splitlines()

    mapping, skipped = {}, []
    for a, b in gate_failures():
        key = str(a) if a == b else f"{a}-{b}"
        got, why = relocate(old_lines, new_lines, a, b)
        if got is None:
            skipped.append((key, why))
            continue
        val = str(got[0]) if a == b else f"{got[0]}-{got[1]}"
        (mapping.setdefault(key, val) if val != key else None)

    for key, val in mapping.items():
        print(f"  {PLAN}:{key} -> :{val}")
    for key, why in skipped:
        print(f"  SKIP {PLAN}:{key} -- {why}", file=sys.stderr)

    if not apply:
        print(f"{len(mapping)} would move, {len(skipped)} need a human "
              f"(re-run with --apply to write)")
        return 0

    files = subprocess.run(
        ["git", "-C", str(ROOT), "ls-files", *CITER_GLOBS],
        capture_output=True, text=True, check=True).stdout.split()
    total = 0
    for rel in files:
        path = ROOT / rel
        try:
            text = original = path.read_text(encoding="utf-8")
        except (UnicodeDecodeError, OSError):
            continue
        for key, val in mapping.items():
            pat = re.compile(r"(?<![!\d])" + re.escape(f"{PLAN}:{key}") + r"(?![\d-])")
            text, n = pat.subn(f"{PLAN}:{val}", text)
            total += n
        if text != original:
            if text.count("\n") != original.count("\n"):
                fail(f"{rel}: rewrite changed the line count")
            path.write_text(text, encoding="utf-8")
    print(f"repointed {total} citation(s) across {len(mapping)} target(s)")
    if skipped:
        print(f"{len(skipped)} target(s) skipped -- resolve those by hand",
              file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
