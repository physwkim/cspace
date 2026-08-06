#!/bin/bash
# Checks PORTING-PLAN.md's `### 감사 완료 조건 현황표` -- the audit layer's
# completion condition, the sibling of `### 완료 조건 현황표` that
# check-phase-status.sh checks.
#
# Why this exists: the Phase table has had a written, verifiable completion
# condition since the plan's first draft, and the audit layer that produces
# its evidence had NONE until 2026-08-07. Rounds therefore ran with no
# stopping condition, and two defects (§308) were found in a tree where every
# gate was green. A condition nobody wrote down cannot be reached, and cannot
# be shown to have been reached.
#
# Named `check-*` so `ci.yml`'s glob runs it, and every input it reads is
# tracked in this repository: no upstream checkout, no docker, no cargo.
#
# A2 and A3 are RE-MEASURED here, not read back. A1 needs an upstream
# checkout and an armed run of two opt-in gates; A4 needs a record spanning
# several rounds. This gate can measure neither, and says so in its own
# output rather than reporting four rows checked -- an aggregate that counts
# an unmeasured row as a passing one is how "26/26 verify scripts passed"
# came to include two scripts that printed "this is not a pass".
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"

. "$REPO_ROOT/tools/ci/gate-lib.sh"

require_caller_tree "$REPO_ROOT"
DOC="$REPO_ROOT/PORTING-PLAN.md"

if [[ ! -s "$DOC" ]]; then
  echo "FAIL $DOC is missing or empty" >&2
  exit 2
fi

python3 - "$REPO_ROOT" <<'PY'
import re
import sys
from pathlib import Path

repo = Path(sys.argv[1])
doc = repo / "PORTING-PLAN.md"
lines = doc.read_text(encoding="utf-8").split("\n")

VERDICTS = {"MET", "UNMET", "PARTIAL", "UNMEASURED"}
# Lives under §308, not beside §5's Phase table, because inserting mid-file
# shifts every line citation below the insertion -- see §308.4.
TABLE_HEADING = "### 308.4 감사 완료 조건 현황표"
# The four rows are fixed by construction: a condition dropped from the table
# is a condition nobody has to meet any more, so a missing id fails rather
# than shrinking the corpus. Same discipline as doc/port-coverage.md's five
# named lines.
REQUIRED = ("A1", "A2", "A3", "A4")
ROW_RE = re.compile(
    r"^\|\s*(A\d)\s+(.+?)\s*\|\s*(.+?)\s*\|\s*(\S+)\s*\|\s*(\S+)\s*\|\s*(\S+)\s*\|$"
)
HEADING_RE = re.compile(r"^#{2,6}\s+")
# Same two heading spellings check-phase-status.sh accepts (`### §226.4` and
# `### 221.1` both exist in this file today).
SECTION_RE = re.compile(r"^#{2,6}\s+§?(\d+(?:\.\d+)*)\b")

failures = []


def fail(line_no, msg):
    failures.append(f"{doc.name}:{line_no}: {msg}")


sections = set()
for line in lines:
    m = SECTION_RE.match(line)
    if m:
        sections.add(m.group(1))

# Locate the table, ending the region on ANY heading -- a `###` accidentally
# inserted mid-table must not silently fold rows out of the checked region.
start = None
for i, line in enumerate(lines, 1):
    if line.strip() == TABLE_HEADING:
        start = i
        break
if start is None:
    print(f"FAIL {doc.name}: `{TABLE_HEADING}` is gone -- the audit layer's "
          "completion condition cannot be deleted to avoid being checked.",
          file=sys.stderr)
    raise SystemExit(1)

rows = {}
for i in range(start, len(lines)):
    line = lines[i]
    if HEADING_RE.match(line):
        break
    m = ROW_RE.match(line)
    if not m:
        continue
    rid, name, clause, verdict, section, date = m.groups()
    line_no = i + 1
    if rid in rows:
        fail(line_no, f"duplicate row for {rid} (first at line {rows[rid][0]})")
        continue
    rows[rid] = (line_no, name, clause, verdict, section, date)
    if verdict not in VERDICTS:
        fail(line_no, f"{rid} has verdict {verdict!r}, not one of {sorted(VERDICTS)}")
    if not section.startswith("§"):
        fail(line_no, f"{rid} cites {section!r}, which is not a `§` reference")
    elif section.lstrip("§") not in sections:
        fail(line_no, f"{rid} cites {section}, no such heading exists in {doc.name}")

for rid in REQUIRED:
    if rid not in rows:
        fail(start, f"{rid} has no row -- all of {', '.join(REQUIRED)} are required")

# ---- A2, re-measured -------------------------------------------------------
# Every citation's class lives in one of these three frozen baselines, one
# citation per row, class field last. A class field carries several classes
# space-separated and a `*N` multiplicity, so membership is tested per token.
BASELINES = (
    "doc/citation-classes.txt",
    "doc/citation-classes-in-repo.txt",
    "doc/upstream-citation-classes.txt",
)
FAILING_CLASSES = {
    "unresolvable",
    "out-of-bounds",
    "blank-line",
    "anchor-mismatch",
    "span-mismatch",
    "section-mismatch",
    "obsolete-header",
}
a2_hits = []
for rel in BASELINES:
    path = repo / rel
    if not path.exists():
        failures.append(f"{rel}: missing -- A2 cannot be measured without it")
        continue
    for line_no, line in enumerate(path.read_text(encoding="utf-8").split("\n"), 1):
        if not line or line.startswith("#"):
            continue
        parts = line.split("\t")
        if len(parts) < 2:
            continue
        for tok in parts[-1].split():
            if tok.split("*", 1)[0] in FAILING_CLASSES:
                a2_hits.append(f"{rel}:{line_no}: {parts[-2]} ({tok})")

# ---- A3, re-measured -------------------------------------------------------
census = repo / "doc/residual-claims-census.md"
a3_open = 0
if not census.exists():
    failures.append(f"{census.name}: missing -- A3 cannot be measured without it")
else:
    for line in census.read_text(encoding="utf-8").split("\n"):
        if line.rstrip().endswith("| OPEN |"):
            a3_open += 1

measured = {"A2": "MET" if not a2_hits else "UNMET",
            "A3": "MET" if a3_open == 0 else "UNMET"}
for rid, want in measured.items():
    if rid not in rows:
        continue
    line_no, _, _, verdict, _, _ = rows[rid]
    if verdict != want:
        fail(line_no, f"{rid} records {verdict}, but this gate measures {want}")

if failures:
    for f in failures:
        print(f"FAIL {f}", file=sys.stderr)
    raise SystemExit(1)

print(f"OK {doc.name} `{TABLE_HEADING}`: {len(rows)} rows, verdict vocabulary and "
      f"§ citations resolve. Re-measured 2 of 4: "
      f"A2 = {measured['A2']} ({len(a2_hits)} failing-class citation(s) across "
      f"{len(BASELINES)} baselines), A3 = {measured['A3']} ({a3_open} OPEN). "
      f"A1 and A4 are NOT measured here -- A1 needs an upstream checkout and an "
      f"armed run of the two opt-in gates, A4 needs a record spanning rounds; "
      f"their rows are checked for shape only, not for truth.")
if a2_hits:
    print("A2's failing-class citations:")
    for h in a2_hits:
        print(f"  {h}")
PY
