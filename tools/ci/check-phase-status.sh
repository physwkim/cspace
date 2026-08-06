#!/bin/bash
# Checks PORTING-PLAN.md's canonical Phase-completion-status table (the
# `### 완료 조건 현황표` subsection under `## 5. 단계별 계획`) for internal
# consistency. That table exists specifically because the status used to be
# a chain of appended "정정" paragraphs a reader had to replay in full to
# learn today's verdict, with newer measurements not reflected in older
# summaries. A drifted or malformed row in the table reintroduces exactly
# that problem for whichever Phase it names.
#
# Named `check-*` so `ci.yml`'s glob runs it: this needs nothing but python3
# and the file itself -- no docker, no cargo, no ROS.
#
# Every failure mode below is a hard failure, never a skip -- see
# check-upstream-bugs-index.sh's own comment on why a heading-region flag
# that clears only on one specific heading level (`## `) silently stays set
# across every finer-grained heading beneath it, so content far below the
# tracked region gets misread as still inside it. This script ends/reassesses
# its own tracked table region on ANY heading line, not just `## `, for the
# same reason -- a stray `### ` subsection accidentally inserted after the
# table must not get silently folded into it, or silently exempt real rows
# below a later `## ` from ever being seen.
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"

. "$REPO_ROOT/tools/ci/gate-lib.sh"

require_caller_tree "$REPO_ROOT"
DOC="$REPO_ROOT/PORTING-PLAN.md"

if [[ ! -s "$DOC" ]]; then
  echo "FAIL $DOC is missing or empty" >&2
  exit 2
fi

python3 - "$DOC" <<'PY'
import re
import sys

path = sys.argv[1]
with open(path, encoding="utf-8") as handle:
    lines = handle.read().split("\n")

VERDICTS = {"MET", "UNMET", "PARTIAL", "UNMEASURED"}
TABLE_HEADING = "### 완료 조건 현황표"

# Newer round sections write their subsection ids as `§NNN.M` and older ones
# as bare `NNN.M` (e.g. `### §226.4 ...` vs `### 221.1 ...`) -- both forms
# exist in the file today. A table citation is checked against this set with
# its own leading `§` stripped, so either heading spelling resolves it; the
# inconsistency is in the document's own headings, not something this check
# should paper over by picking one form and failing the other's citations.
# A real fence-open/close line is 3+ backticks (optionally with a trailing
# language tag) and NOTHING ELSE with a backtick in it. This file also uses
# 4-backtick INLINE code spans mid-paragraph to quote literal triple-backtick
# text while discussing fence conventions (e.g. "```` ```text ````이고") --
# those lines start with backticks too, so a naive `startswith("```")` scan
# misreads them as block-fence toggles and desyncs in_fence for the rest of
# the file. Requiring no further backtick on the line rules those out.
FENCE_RE = re.compile(r"^ {0,3}`{3,}[^`]*$")
HEADING_ID_RE = re.compile(r"^#{1,6}\s*§?(\d+(?:\.\d+)?)\b")
PHASE_HEADING_RE = re.compile(r"^### Phase (\d+) ")
PHASE_ROW_START_RE = re.compile(r"^\|\s*Phase\s+(\d+)\s*\|")
ROW_RE = re.compile(
    r"^\|\s*Phase\s+(\d+)\s*\|\s*(.+?)\s*\|\s*(\S+)\s*\|\s*(\S+)\s*\|\s*(\S+)\s*\|$"
)

failures = []


def fail(line_no, message):
    failures.append(f"{path}:{line_no}: {message}")


# Conflict markers, including the diff3 base marker: a scan for only the
# three familiar ones reports a file clean while `|||||||` sits in it.
for i, line in enumerate(lines, 1):
    for marker in ("<<<<<<<", "|||||||", "=======", ">>>>>>>"):
        if line.startswith(marker):
            fail(i, f"unresolved conflict marker {marker!r}")

heading_ids = set()
phase_heading_lines = {}  # phase number -> line of its `### Phase N` heading

in_fence = False
in_table = False
table_rows = []  # (line_no, phase, clause, verdict, section_id, date)

for i, line in enumerate(lines, 1):
    if FENCE_RE.match(line):
        in_fence = not in_fence
        continue
    if in_fence:
        continue

    if line.startswith("#"):
        match = HEADING_ID_RE.match(line)
        if match is not None:
            heading_ids.add(match.group(1))
        phase_match = PHASE_HEADING_RE.match(line)
        if phase_match is not None:
            phase_heading_lines.setdefault(phase_match.group(1), i)
        # A heading of ANY level ends/reassesses the table region.
        in_table = line.strip() == TABLE_HEADING
        continue

    if in_table and PHASE_ROW_START_RE.match(line):
        match = ROW_RE.match(line)
        if match is None:
            fail(i, f"table row does not match the 5-column shape: {line}")
        else:
            phase, clause, verdict, section_id, date = match.groups()
            table_rows.append((i, phase, clause, verdict, section_id, date))

if in_fence:
    fail(len(lines), "unterminated ``` fence -- rows after it were not read")

# A zero count is the failure this whole script exists to make loud: an
# empty parse and a clean file are otherwise the same exit code.
if not table_rows:
    fail(1, "parsed zero table rows -- the table grammar changed and this checked nothing")
if not phase_heading_lines:
    fail(1, "parsed zero '### Phase N' headings -- §5's own Phase list changed shape")

for line_no, phase, clause, verdict, section_id, date in table_rows:
    if verdict not in VERDICTS:
        fail(
            line_no,
            f"Phase {phase} row has verdict {verdict!r}, not one of {sorted(VERDICTS)}",
        )
    cited = section_id.lstrip("§")
    if cited not in heading_ids:
        fail(
            line_no,
            f"Phase {phase} row cites {section_id!r}, no such heading exists in the file",
        )

seen = {}
for line_no, phase, clause, verdict, section_id, date in table_rows:
    key = (phase, clause)
    if key in seen:
        fail(line_no, f"duplicate row for Phase {phase} / {clause!r} (first at line {seen[key]})")
    else:
        seen[key] = line_no

rows_by_phase = {phase for _, phase, *_ in table_rows}
for phase, heading_line in sorted(phase_heading_lines.items(), key=lambda kv: int(kv[0])):
    if phase not in rows_by_phase:
        fail(heading_line, f"Phase {phase} has a '### Phase {phase}' heading in §5 but no table row")

if failures:
    for failure in failures:
        print("FAIL " + failure, file=sys.stderr)
    sys.exit(1)

print(
    f"OK PORTING-PLAN.md: {len(table_rows)} status rows across "
    f"{len(rows_by_phase)} phases (of {len(phase_heading_lines)} defined) "
    "agree on verdict vocabulary, every cited § resolves to a real heading, "
    "and no phase or row is missing/duplicated"
)
PY
