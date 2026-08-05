#!/bin/bash
# Checks that every `§N` / `§N.M` reference in a tracked file resolves to a
# heading that actually exists.
#
# Nothing checked this before, and three references had rotted into numbers no
# heading ever carried, while four subsections of §177 existed only as bold
# labels inside a paragraph, so a citation to the first of them from
# `doc/claim-audit/moveit-kinematics.md` pointed at nothing. (The dead numbers
# are named in `0880ea5`, not here: this file is scanned like any other, so
# quoting one as prose would make the gate fail on its own header -- which is
# exactly how this line came to be written twice.)
# Both shapes read as fine to a reviewer scrolling past: the number is
# plausible, the prose around it is true, and the reader who follows it lands
# somewhere unrelated or nowhere at all. A section number is the only handle
# this port has for "the round that measured this" -- 2,800 of them across 587
# files -- so a dangling one silently detaches a claim from its evidence.
#
# Resolution rule, deliberately narrow:
#
#   1. PORTING-PLAN.md's headings are the default namespace. Both spellings the
#      file actually uses are headings: `## §221` and bare `### 221.1`.
#   2. A file with its own numbered headings resolves its own references
#      against them first (the claim-audit and ledger docs number their own
#      sections).
#   3. A reference on a line that *names another tracked markdown file* also
#      resolves against that file's headings. This is the only cross-file form
#      in the tree (`... `ros/moveit-ros/doc/message-mapping.md` §17.5 ...`),
#      and the window is one line, not a paragraph, so the exemption cannot be
#      earned by a filename mentioned somewhere nearby.
#   4. Anything else must be listed in `section-reference-external.json` with a
#      reason. That file exists for genuine non-plan section numbers -- today,
#      a textbook.
#
# Named `check-*` so `ci.yml`'s glob runs it: python3 and the tree, nothing
# else. A parse that finds zero headings or zero references fails rather than
# passing, and the counts are printed on the OK line, because a checker that
# quietly stopped seeing the file spells itself the same as a clean tree.
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$REPO_ROOT"

python3 - "$REPO_ROOT/tools/ci/section-reference-external.json" <<'PY'
import json
import os
import re
import subprocess
import sys

allow_path = sys.argv[1]
with open(allow_path, encoding="utf-8") as handle:
    allow = json.load(handle)["external"]
if not allow:
    print(f"FAIL {allow_path} lists no entries -- delete it or the rule it encodes", file=sys.stderr)
    sys.exit(2)
allow_matches = [entry["match"] for entry in allow]

SCANNED_SUFFIXES = ("md", "rs", "sh", "py", "toml", "yml", "yaml", "json")
HEADING_RE = re.compile(r"^#{2,6}\s+§?(\d+(?:\.\d+)*)\b")
REF_RE = re.compile(r"§(\d+(?:\.\d+)*)")

tracked = subprocess.run(
    ["git", "ls-files"], capture_output=True, text=True, check=True
).stdout.split("\n")
tracked = [p for p in tracked if p]


def read(path):
    with open(path, encoding="utf-8", errors="replace") as handle:
        return handle.read().split("\n")


def headings_of(path):
    return {m.group(1) for line in read(path) if (m := HEADING_RE.match(line))}


markdown = [p for p in tracked if p.endswith(".md")]
by_file = {p: headings_of(p) for p in markdown}
by_basename = {os.path.basename(p): p for p in markdown}

plan = by_file.get("PORTING-PLAN.md", set())
if not plan:
    print("FAIL PORTING-PLAN.md yielded zero numbered headings -- the heading "
          "grammar changed and this checked nothing", file=sys.stderr)
    sys.exit(1)

failures = []
references = 0
files_with_refs = 0

for path in tracked:
    if path.rsplit(".", 1)[-1] not in SCANNED_SUFFIXES:
        continue
    lines = read(path)
    own = by_file.get(path, set())
    seen_here = False
    for line_no, line in enumerate(lines, 1):
        if not REF_RE.search(line):
            continue
        if any(needle in line for needle in allow_matches):
            continue
        # Rule 3: only a filename on this very line widens the namespace.
        cross = set()
        for basename, other in by_basename.items():
            if basename in line:
                cross |= by_file[other]
        for match in REF_RE.finditer(line):
            references += 1
            seen_here = True
            number = match.group(1)
            if number in plan or number in own or number in cross:
                continue
            failures.append(
                f"{path}:{line_no}: §{number} resolves to no heading -- "
                f"{line.strip()[:100]}"
            )
    if seen_here:
        files_with_refs += 1

if not references:
    print("FAIL parsed zero '§N' references across the tree -- the reference "
          "grammar changed and this checked nothing", file=sys.stderr)
    sys.exit(1)

if failures:
    for failure in failures:
        print("FAIL " + failure, file=sys.stderr)
    print(
        f"\n{len(failures)} dangling reference(s). Point each at the section that "
        "holds the content, or -- if the content lives in a paragraph rather than "
        "a heading -- promote that paragraph to a real heading. Do NOT add it to "
        f"{os.path.relpath(allow_path)}; that file is for section numbers belonging "
        "to something outside this repository.",
        file=sys.stderr,
    )
    sys.exit(1)

print(
    f"OK {references} '§N' references across {files_with_refs} tracked files all "
    f"resolve to a heading ({len(plan)} in PORTING-PLAN.md, plus per-file and "
    f"same-line cross-file namespaces; {len(allow)} vetted external exemption(s))"
)
PY
