#!/bin/bash
# Every number this repository publishes about the Phase 8 500-problem baseline
# must be reproducible from `doc/phase8-baseline-500/`'s committed NDJSONs.
#
# The four tables in §269.3, §269.4, §286.3 and §286.6 were computed from
# scratch files that were never committed: `git ls-files` found only
# `doc/phase8-condition2-stomp/`, one round's port STOMP arms and no C++ arm at
# all. So a reader who wanted to check a cell could not, and an edit to one
# could not be caught. Re-running the arms is not the fix -- the port STOMP
# side alone is over an hour per config and needs the oracle container. The fix
# is that the inputs are committed and this gate re-derives the report from them
# in under a second, with no oracle, no docker, no cargo and no upstream
# checkout, which is exactly what `.github/workflows/ci.yml`'s `check-*` glob
# provides.
#
# `rederive.py --check` compares its whole report against the committed
# transcript, so a number that drifts fails here by name rather than by a total.
#
#   tools/ci/check-phase8-baseline-500.sh
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"

. "$REPO_ROOT/tools/ci/gate-lib.sh"

require_caller_tree "$REPO_ROOT"
cd "$REPO_ROOT"

DIR="doc/phase8-baseline-500"

# Named one by one rather than globbed: a glob reports OK over whatever
# survives, and the failure this directory exists to prevent is a file going
# missing without the report noticing.
WANT=(
  "$DIR/floor_wall.250.900001.set.json"
  "$DIR/cage.250.900002.set.json"
  "$DIR/seed.floor_wall.ndjson"
  "$DIR/seed.cage.ndjson"
  "$DIR/port.chomp.floor_wall.ndjson"
  "$DIR/port.chomp.cage.ndjson"
  "$DIR/port.stomp.floor_wall.ndjson"
  "$DIR/port.stomp.cage.ndjson"
  "$DIR/cpp.chomp.floor_wall.ndjson"
  "$DIR/cpp.chomp.cage.ndjson"
  "$DIR/cpp.stomp.floor_wall.ndjson"
  "$DIR/cpp.stomp.cage.ndjson"
  "$DIR/repeat.cpp.chomp.floor_wall.ndjson"
  "$DIR/repeat.cpp.stomp.floor_wall.ndjson"
  "$DIR/rederive.py"
  "$DIR/rederive.txt"
  "$DIR/README.md"
)

missing=0
for f in "${WANT[@]}"; do
  if ! git ls-files --deduplicate --error-unmatch "$f" >/dev/null 2>&1; then
    echo "FAIL $f is not tracked -- the baseline is not committed" >&2
    missing=$((missing + 1))
  fi
done
if [ "$missing" -ne 0 ]; then
  echo "FAIL $missing of ${#WANT[@]} baseline files are untracked" >&2
  exit 1
fi

# Not piped: a pipeline reports the filter's status, which is how a drifted
# number becomes a silent pass.
"$DIR/rederive.py" --check "$DIR/rederive.txt"
