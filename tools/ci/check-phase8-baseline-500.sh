#!/bin/bash
# Every number the Phase 8 STOMP/CHOMP plan sections publish must be
# reproducible from the NDJSONs committed beside them, across all three
# sibling directories: doc/phase8-baseline-500/, doc/phase8-condition2-stomp/
# and doc/phase8-seedbase-stomp/.
#
# The four tables in §269.3, §269.4, §286.3 and §286.6 were computed from
# scratch files that were never committed: `git ls-files` found only
# `doc/phase8-condition2-stomp/`, one round's port STOMP arms and no C++ arm at
# all. So a reader who wanted to check a cell could not, and an edit to one
# could not be caught. `doc/phase8-baseline-500/` fixed that for itself, but
# its two siblings shared the same gap -- only PORTING-PLAN.md prose recorded
# that anyone had ever run their `rederive.py`, and nothing re-ran it. The
# three directories share one shape (a fixed file list, `rederive.py --check`
# against a committed `rederive.txt`), so one gate walks all three instead of
# three near-copies of the same loop.
#
# Re-running the arms themselves is not the fix -- the port STOMP side alone
# is over an hour per config and needs the oracle container. The fix is that
# the inputs are committed and this gate re-derives each directory's report
# from them in under a second, with no oracle, no docker, no cargo and no
# upstream checkout, which is exactly what `.github/workflows/ci.yml`'s
# `check-*` glob provides.
#
# `rederive.py --check` compares its whole report against the committed
# transcript, so a number that drifts fails here by name rather than by a
# total.
#
#   tools/ci/check-phase8-baseline-500.sh
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"

. "$REPO_ROOT/tools/ci/gate-lib.sh"

require_caller_tree "$REPO_ROOT"
cd "$REPO_ROOT"

# Each directory's file list, named one by one rather than globbed: a glob
# reports OK over whatever survives, and the failure these directories exist
# to prevent is a file going missing without the report noticing.
#
# floor_wall.stats and cage.stats were tracked but absent from this list
# before this gate covered more than one directory -- `git ls-files` counts
# 19 files here, one more than the 17 named below at the time this gate named
# only this directory. Fixed here rather than left, since the gap is the
# exact defect this list exists to prevent.
WANT_phase8_baseline_500=(
  floor_wall.250.900001.set.json
  cage.250.900002.set.json
  seed.floor_wall.ndjson
  seed.cage.ndjson
  floor_wall.stats
  cage.stats
  port.chomp.floor_wall.ndjson
  port.chomp.cage.ndjson
  port.stomp.floor_wall.ndjson
  port.stomp.cage.ndjson
  cpp.chomp.floor_wall.ndjson
  cpp.chomp.cage.ndjson
  cpp.stomp.floor_wall.ndjson
  cpp.stomp.cage.ndjson
  repeat.cpp.chomp.floor_wall.ndjson
  repeat.cpp.stomp.floor_wall.ndjson
  rederive.py
  rederive.txt
  README.md
)

WANT_phase8_condition2_stomp=(
  floor_wall.250.900001.set.json
  cage.250.900002.set.json
  seed.floor_wall.ndjson
  seed.cage.ndjson
  base.port.stomp.floor_wall.ndjson
  base.port.stomp.cage.ndjson
  base.floor_wall.stats
  base.cage.stats
  base.floor_wall.cost.txt
  base.cage.cost.txt
  mut.port.stomp.floor_wall.ndjson
  mut.port.stomp.cage.ndjson
  mutation-collision-penalty-zero.patch
  revert-subset.floor_wall.0-19.ndjson
  run-subset.sh
  rederive.py
  rederive.txt
  README.md
)

WANT_phase8_seedbase_stomp=(
  floor_wall.250.900001.set.json
  cage.250.900002.set.json
  floor_wall.stats
  cage.stats
  port.stomp.floor_wall.ndjson
  port.stomp.cage.ndjson
  stomp.floor_wall.ndjson
  stomp.cage.ndjson
  cpp700001.floor_wall.ndjson
  cpp700001.cage.ndjson
  rederive.py
  rederive.txt
  README.md
)

# dir:want-array-name pairs. One loop derives the target list from this table
# instead of a script per directory.
DIRS=(
  "doc/phase8-baseline-500:WANT_phase8_baseline_500"
  "doc/phase8-condition2-stomp:WANT_phase8_condition2_stomp"
  "doc/phase8-seedbase-stomp:WANT_phase8_seedbase_stomp"
)

total_missing=0
for entry in "${DIRS[@]}"; do
  dir="${entry%%:*}"
  declare -n want="${entry#*:}"
  dir_missing=0
  for f in "${want[@]}"; do
    if ! git ls-files --deduplicate --error-unmatch "$dir/$f" >/dev/null 2>&1; then
      echo "FAIL $dir/$f is not tracked -- the evidence is not committed" >&2
      dir_missing=$((dir_missing + 1))
    fi
  done
  if [ "$dir_missing" -ne 0 ]; then
    echo "FAIL $dir_missing of ${#want[@]} $dir files are untracked" >&2
  fi
  total_missing=$((total_missing + dir_missing))
  unset -n want
done
if [ "$total_missing" -ne 0 ]; then
  exit 1
fi

# Not piped: a pipeline reports the filter's status, which is how a drifted
# number becomes a silent pass.
for entry in "${DIRS[@]}"; do
  dir="${entry%%:*}"
  "$dir/rederive.py" --check "$dir/rederive.txt"
done
