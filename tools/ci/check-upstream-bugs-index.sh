#!/bin/bash
# Checks `doc/upstream-bugs.md` against its own "Entry format" contract: the
# `## Index` table and the `###` entries must name the same slugs, in the same
# order, with the same status, and every entry must carry the fields the
# contract lists.
#
# Nothing checked this file before. It is append-only by design -- "Append
# anywhere; never rename a slug once it is cited" -- which is exactly the
# shape that produces an append/append conflict every time two branches add an
# entry in one round, and a hand-resolved conflict is where an Index row goes
# missing or lands out of order. Resolving one by hand and eyeballing "the
# counts match" is not a check: two rows can go wrong in opposite directions
# and still total right. The `## Index` table is also the only place a reader
# sees every status at once, so a row that drifts from its entry misreports
# the port's actual position on an upstream bug.
#
# Named `check-*` so `ci.yml`'s glob runs it: this needs nothing but python3
# and the file itself -- no docker, no cargo, no ROS.
#
# Every failure mode below is a hard failure, never a skip. A heading this
# script cannot parse fails rather than being passed over, because "the
# grammar changed and the checker quietly stopped seeing half the file" is the
# one outcome that spells itself the same as success. The counts are printed
# on the OK line for the same reason -- a run that checked nothing must not
# look like a run that checked everything.
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
DOC="$REPO_ROOT/doc/upstream-bugs.md"

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

STATUSES = {"not-reproduced", "reproduced-deliberately", "reproduced-grandfathered"}
# The contract's own field list. `Deviation` is deliberately absent: it was
# added to the format after most entries were written and sits on 18 of them,
# so requiring it would fail entries that predate it rather than catch drift.
REQUIRED = ["Upstream", "Port", "Symptom", "Evidence"]
# A `reproduced-deliberately` entry answers the opposite question, and one
# entry states it that way on purpose. Exactly one of the two must be present.
COST_FIELDS = ["Cost of not reproducing", "Cost of reproducing"]

ENTRY_RE = re.compile(r"^### `([a-z0-9-]+)` — .+ — ([a-z-]+)$")
INDEX_ROW_RE = re.compile(r"^\| `([a-z0-9-]+)` \| ([a-z-]+) \|$")

failures = []


def fail(line_no, message):
    failures.append(f"{path}:{line_no}: {message}")


# Conflict markers, including the diff3 base marker: a scan for only the three
# familiar ones reports a file clean while `|||||||` sits in it.
for i, line in enumerate(lines, 1):
    for marker in ("<<<<<<<", "|||||||", "=======", ">>>>>>>"):
        if line.startswith(marker):
            fail(i, f"unresolved conflict marker {marker!r}")

# Fenced blocks hold the format template, whose `### `<slug>`` line is a
# placeholder and not an entry. Track fences rather than filtering the
# placeholder by name, so a second template block does not need a new special
# case here.
in_fence = False
entries = []  # (line_no, slug, status)
index_rows = []  # (line_no, slug, status)
index_heading_lines = []
in_index = False

for i, line in enumerate(lines, 1):
    if line.startswith("```"):
        in_fence = not in_fence
        continue
    if in_fence:
        continue

    if line.startswith("## "):
        in_index = line.strip() == "## Index"
        if in_index:
            index_heading_lines.append(i)
        continue

    if line.startswith("### "):
        match = ENTRY_RE.match(line)
        if match is None:
            fail(
                i,
                "entry heading does not match "
                "'### `<slug>` — <symptom> — <status>': " + line,
            )
        else:
            entries.append((i, match.group(1), match.group(2)))
        continue

    if in_index and line.startswith("| `"):
        match = INDEX_ROW_RE.match(line)
        if match is None:
            fail(i, "Index row does not match '| `<slug>` | <status> |': " + line)
        else:
            index_rows.append((i, match.group(1), match.group(2)))

if in_fence:
    fail(len(lines), "unterminated ``` fence -- entries after it were not read")

if len(index_heading_lines) != 1:
    fail(
        index_heading_lines[0] if index_heading_lines else 1,
        f"expected exactly one '## Index' heading, found {len(index_heading_lines)}",
    )

# A zero count is the failure this whole script exists to make loud: an empty
# parse and a clean file are otherwise the same exit code.
if not entries:
    fail(1, "parsed zero entries -- the '###' grammar changed and this checked nothing")
if not index_rows:
    fail(1, "parsed zero Index rows -- the table grammar changed and this checked nothing")

for line_no, slug, status in entries + index_rows:
    if status not in STATUSES:
        fail(line_no, f"`{slug}` has status {status!r}, not one of {sorted(STATUSES)}")

for label, items in (("entry", entries), ("Index row", index_rows)):
    seen = {}
    for line_no, slug, _ in items:
        if slug in seen:
            fail(line_no, f"duplicate {label} `{slug}` (first at line {seen[slug]})")
        else:
            seen[slug] = line_no

entry_slugs = [slug for _, slug, _ in entries]
index_slugs = [slug for _, slug, _ in index_rows]

for slug in index_slugs:
    if slug not in entry_slugs:
        line_no = next(n for n, s, _ in index_rows if s == slug)
        fail(line_no, f"Index row `{slug}` has no matching entry")
for slug in entry_slugs:
    if slug not in index_slugs:
        line_no = next(n for n, s, _ in entries if s == slug)
        fail(line_no, f"entry `{slug}` has no Index row")

if entry_slugs != index_slugs and sorted(entry_slugs) == sorted(index_slugs):
    for position, (want, got) in enumerate(zip(index_slugs, entry_slugs)):
        if want != got:
            fail(
                index_rows[position][0],
                f"Index and entries diverge in order at position {position + 1}: "
                f"Index has `{want}`, entries have `{got}`",
            )
            break

status_by_slug = {slug: status for _, slug, status in index_rows}
for line_no, slug, status in entries:
    indexed = status_by_slug.get(slug)
    if indexed is not None and indexed != status:
        fail(
            line_no,
            f"`{slug}` heading says {status!r} but its Index row says {indexed!r}",
        )

# Field checks need each entry's body: from its own heading to the next `###`
# or `##`, whichever comes first, so the trailing prose sections are excluded.
boundaries = [line_no for line_no, _, _ in entries]
section_starts = [i for i, line in enumerate(lines, 1) if line.startswith("## ")]
for position, (line_no, slug, status) in enumerate(entries):
    end = len(lines)
    for candidate in boundaries[position + 1 :] + section_starts:
        if candidate > line_no:
            end = min(end, candidate - 1)
    body = "\n".join(lines[line_no:end])
    present = set(re.findall(r"^\*\*([A-Za-z ]+):\*\*", body, re.M))

    for field in REQUIRED:
        if field not in present:
            fail(line_no, f"`{slug}` has no '**{field}:**' line")

    costs = [field for field in COST_FIELDS if field in present]
    if len(costs) != 1:
        fail(
            line_no,
            f"`{slug}` must carry exactly one of {COST_FIELDS}, found {costs}",
        )

    # Two grammars are live here. The template's is `**Status:** `<status>`.`
    # and every entry written since carries it; the older entries state the
    # status as prose ("already not reproduced", "reproduced deliberately").
    # Both are accepted, but both must *say* the heading's status: the check
    # is on agreement, not on phrasing, because the phrasing carries the
    # reasoning and rewriting 25 entries to satisfy a checker would lose it.
    # The backticked form is held to exact equality, so new entries cannot
    # drift into a third grammar.
    status_line = re.search(r"^\*\*Status:\*\*(.*)$", body, re.M)
    if status_line is None:
        fail(line_no, f"`{slug}` has no '**Status:**' line")
    else:
        text = status_line.group(1)
        quoted = re.match(r"\s+`([a-z-]+)`", text)
        if quoted is not None:
            if quoted.group(1) != status:
                fail(
                    line_no,
                    f"`{slug}` body says status {quoted.group(1)!r} "
                    f"but its heading says {status!r}",
                )
        else:
            # Compare on letters alone, so "already not reproduced" and
            # "reproduced deliberately" both match their hyphenated heading.
            flattened = re.sub(r"[^a-z]", "", text.lower())
            if status.replace("-", "") not in flattened:
                fail(
                    line_no,
                    f"`{slug}` heading says {status!r} but its '**Status:**' line "
                    f"does not say so: {text.strip()!r}",
                )

if failures:
    for failure in failures:
        print("FAIL " + failure, file=sys.stderr)
    sys.exit(1)

print(
    f"OK doc/upstream-bugs.md: {len(entries)} entries and {len(index_rows)} Index rows "
    "agree on slug, order and status; every entry carries its contract fields"
)
PY
