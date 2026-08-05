#!/bin/bash
# Checks that `PORTING-PLAN.md`'s section numbers are unique, and that no
# branch has left a `§NEW` placeholder behind.
#
# Why this exists: parallel panels each append a section and each picks the
# next free number by reading the file in its own worktree. Every branch is
# therefore correct alone and wrong together -- in one round three separate
# branches all chose `§226` and a fourth pair both chose `§220`. Git merges
# them without complaint because the appends do not overlap textually, so the
# collision is invisible until a reader follows a cross-reference and lands in
# another panel's section. Renumbering afterwards is worse than it sounds: the
# references live in Rust doc comments and other docs too, and the round that
# renumbered `§226 -> §227` in the Markdown left four references in
# `moveit-planners-pilz/src/lib.rs` pointing at the *other* panel's §226,
# which reads as correct because that heading exists.
#
# The convention this enforces: a worker writes `## §NEW <title>` (and
# `### §NEW.1`, `§NEW2` for a second section), never a number. The number is
# assigned by the single agent that can see all branches at once -- the one
# doing the merge. This script is what makes that a rule rather than a habit:
# a duplicate number fails here, and so does a `§NEW` that reached the trunk
# without being assigned.
#
# Named `check-*` so `ci.yml`'s glob runs it: needs nothing but python3 and
# the tracked files.
#
# Every failure mode is a hard failure, never a skip -- including a parse that
# finds zero sections, because "the heading grammar changed and this checked
# nothing" otherwise spells itself exactly like success. The counts are on the
# OK line for the same reason.
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$REPO_ROOT"

if [[ ! -s PORTING-PLAN.md ]]; then
  echo "FAIL PORTING-PLAN.md is missing or empty" >&2
  exit 2
fi

python3 - <<'PY'
import re
import subprocess
import sys

tracked = [p for p in subprocess.run(
    ["git", "ls-files", "-z"], capture_output=True, check=True
).stdout.decode("utf-8").split("\0") if p]

with open("PORTING-PLAN.md", encoding="utf-8") as handle:
    lines = handle.read().split("\n")

# Two heading grammars are live and both are load-bearing. Sections 0-140 were
# written as `## 12. title`; everything from 141 on is `## §141 title`. A
# checker that knew only the newer one would silently skip the first 140
# sections, so both are parsed and a heading matching neither is a failure,
# not a skip.
NUMBERED = re.compile(r"^(#{2,4}) (§?)([0-9]+(?:\.[0-9]+)*)\.? ")

# Fenced blocks quote other documents, so a `## ` inside one is not a section.
# Tracking them by toggling on any line starting with three backticks reads
# this file wrong twice over, and both errors hide sections rather than
# inventing them. First, five paragraphs here *discuss* fences and open with a
# four-backtick inline span (```` ```rust ```` ...); a toggle counts each as a
# fence and the run of backticks is odd, so the flag ends stuck on. Second, a
# block opened with four backticks is not closed by the three-backtick lines
# nested inside it. Together they swallowed 102 of the 229 sections -- the
# checker reported OK having looked at just over half the file.
#
# So: an opening fence is a run of >= 3 backticks whose info string carries no
# backtick of its own, and only a run at least as long, alone on its line,
# closes it. A fence still open at EOF is a hard failure for the same reason.
FENCE = re.compile(r"^(`{3,})([^`]*)$")

top_level = {}   # id -> line number, `##` headings only
all_ids = set()
placeholders = []
failures = []

fence_len = 0
fence_opened_at = 0
for line_no, line in enumerate(lines, 1):
    match = FENCE.match(line)
    if match is not None:
        run, info = len(match.group(1)), match.group(2).strip()
        if fence_len == 0:
            fence_len, fence_opened_at = run, line_no
        elif run >= fence_len and info == "":
            fence_len = 0
        continue
    if fence_len:
        continue
    if "§NEW" in line:
        placeholders.append((line_no, "PORTING-PLAN.md", line.strip()))
    match = NUMBERED.match(line)
    if match is None:
        continue
    hashes, _, section = match.groups()
    all_ids.add(section)
    if len(hashes) != 2:
        continue
    if section in top_level:
        failures.append(
            f"PORTING-PLAN.md:{line_no}: duplicate section §{section} "
            f"(first at line {top_level[section]}) -- two branches picked the "
            f"same number; the merge must renumber one of them"
        )
    else:
        top_level[section] = line_no

# A placeholder anywhere in the tree, not just in the plan: the whole point is
# that the worker writes `§NEW` in its Rust doc comments too, and the merger
# rewrites all of them together.
#
# Prose and Rust only. Sections are cited in documentation and in doc
# comments, never in build or CI scripts -- and the scripts are where a
# `§NEW` is a *mention* rather than a citation, this file being the first
# example: it names the placeholder eight times because it defines it. Scoping
# by file kind keeps that out by rule instead of by an exception list with
# this script's own name in it.
for path in tracked:
    if path == "PORTING-PLAN.md" or not path.endswith((".md", ".rs")):
        continue
    try:
        with open(path, encoding="utf-8") as handle:
            for line_no, line in enumerate(handle, 1):
                if "§NEW" in line:
                    placeholders.append((line_no, path, line.strip()))
    except (OSError, UnicodeDecodeError):
        continue

for line_no, path, text in placeholders:
    failures.append(
        f"{path}:{line_no}: unassigned section placeholder -- the merge must "
        f"replace §NEW with the number it assigns: {text[:90]}"
    )

if fence_len:
    failures.append(
        f"PORTING-PLAN.md:{fence_opened_at}: fence opened here is never closed "
        f"-- every section below it was skipped by this check"
    )

if not top_level:
    failures.append(
        "PORTING-PLAN.md:1: parsed zero `##` sections -- the heading grammar "
        "changed and this script checked nothing"
    )

if failures:
    for failure in failures:
        print("FAIL " + failure, file=sys.stderr)
    sys.exit(1)

print(
    f"OK PORTING-PLAN.md: {len(top_level)} top-level sections, "
    f"{len(all_ids)} numbered headings, all distinct; no §NEW placeholder in "
    f"{len(tracked)} tracked files"
)
PY
