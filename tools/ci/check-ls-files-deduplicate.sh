#!/usr/bin/env bash
# Fails if any script invokes `git ls-files` without `--deduplicate`.
#
# During an unresolved merge the index holds a conflicted path three times
# (stage 1 = base, 2 = ours, 3 = theirs) and plain `git ls-files` prints one
# row per stage; only `--deduplicate` (git >= 2.31) collapses them. Every
# gate here derives its corpus from that list and publishes the corpus size
# as its own evidence of coverage, so a run mid-merge reads each conflicted
# file's contents two or three times and reports a total that is
# indistinguishable from a corpus that grew.
#
# This is the third round on the same anchor, and that is why it is a gate
# rather than a fourth sweep:
#
#   f720642  swept eight scripts, ten sites
#   81babe5  fixed three more (check-citation-drift.py, check-test-doc-links.sh
#            x2), written up in doc/claim-audit/tools-ci-gates.md under the
#            heading "A third gate family" -- all three files were created
#            after f720642
#   here     measure-upstream-citations.py's `tracked_files` and
#            `upstream_tracked`, in a file created at 7035998,
#            43 minutes after f720642. Measured on the p10-phase13 merge:
#            "2728 upstream citations across 349 tracked .md/.rs files" with
#            PORTING-PLAN.md conflicted, 1968 across 347 once resolved -- one
#            conflicted path, +2 file rows, +760 citations.
#
# A sweep cannot cover a file that does not exist yet, and each occurrence so
# far arrived in a file written after the previous sweep. A glob-driven gate
# covers the next one.
#
# Run by CI via the `tools/ci/check-*.sh` glob. Needs no docker.
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$repo_root"

if ! files_raw="$(git ls-files --deduplicate -- '*.sh' '*.py' '*.yml' '*.yaml' | sort)"; then
  echo "FAIL git ls-files failed -- nothing was checked." >&2
  exit 1
fi
files=()
[ -n "$files_raw" ] && mapfile -t files <<<"$files_raw"
if [ "${#files[@]}" -eq 0 ]; then
  echo "FAIL no .sh/.py/.yml file is tracked -- nothing was checked." >&2
  exit 1
fi

# Invocation, not mention. After the `ls-files` token (with any Python list
# punctuation stripped) an invocation's next token is a flag, `--`, a pipe,
# or nothing; a prose mention is followed by an English word, a closing
# backtick, or an elision. Only the invocation shape is judged, so the
# headers above -- which name this very command in prose -- do not trip it.
# Comment lines are skipped outright.
python3 - "${files[@]}" <<'PY'
import re
import sys

TOKEN = re.compile(r"ls-files[\"'\s,]*([^\s]*)")
INVOCATION_NEXT = re.compile(r"^(-|\||\)|$)")

bad = []
seen = 0
for path in sys.argv[1:]:
    try:
        text = open(path, encoding="utf-8").read()
    except (UnicodeDecodeError, FileNotFoundError):
        continue
    for lineno, line in enumerate(text.split("\n"), 1):
        if "ls-files" not in line:
            continue
        if line.lstrip().startswith("#"):
            continue
        for m in TOKEN.finditer(line):
            raw = m.group(1)
            if not INVOCATION_NEXT.match(raw):
                continue  # prose mention, not a command
            seen += 1
            # Python list punctuation only; `-` is never stripped, so `--` and
            # `-z` still read as the flags they are and fail below.
            if raw.strip("\"',[]();") != "--deduplicate":
                bad.append((path, lineno, line.strip()))

if seen == 0:
    # A pattern that matches nothing reads back exactly like a clean tree.
    # This repo has more than a dozen real invocations; zero means TOKEN and
    # the corpus have drifted apart, not that the gate passed.
    print("FAIL matched no `git ls-files` invocation at all -- TOKEN and the "
          "scanned corpus have drifted apart, so this gate would pass on any "
          "tree", file=sys.stderr)
    raise SystemExit(1)

if bad:
    print(f"{len(bad)} `git ls-files` invocation(s) without --deduplicate:", file=sys.stderr)
    for path, lineno, line in bad:
        print(f"  {path}:{lineno}: {line}", file=sys.stderr)
    print(file=sys.stderr)
    print("An unresolved merge makes the index hold a conflicted path once per", file=sys.stderr)
    print("stage, so the corpus -- and the total the gate prints as its own", file=sys.stderr)
    print("coverage evidence -- silently triples. Add --deduplicate.", file=sys.stderr)
    raise SystemExit(1)

print(f"OK {seen} `git ls-files` invocation(s), all --deduplicate")
PY
