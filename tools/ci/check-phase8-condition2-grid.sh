#!/bin/bash
# Every line `analyse-phase8-condition2-grid.py` prints for CHOMP and STOMP
# must be reproducible from the NDJSONs already committed under
# `doc/phase8-baseline-500/`, against a transcript committed beside them.
#
# `analyse-phase8-condition2-grid.py` is Phase 8's condition-2 answer for the
# CHOMP/STOMP class -- see its own module doc: `r*`, the finest grid
# resolution at which the C++ implementation of the SAME planner reaches
# 100%, and whether the port meets condition 2 there. It was written,
# `measure-phase8-condition2-grid.sh` was written to feed it, and neither had
# a caller: `git log --oneline -- tools/ci/analyse-phase8-condition2-grid.py`
# shows only the commit that added it, and nothing in `tools/ci/check-*` or
# `tools/ci/verify-*` ever names it. `check-phase8-baseline-500.sh` protects
# the raw grid NUMBERS its sibling `doc/phase8-baseline-500/rederive.py`
# reads (every resolution present, per-resolution counts unchanged) -- but
# `rederive.py` has no McNemar paired test, no `r*` search and no MET/UNMET
# line of its own; it is a different, narrower check. A bug in THIS script's
# own r*-search or paired-test arithmetic would reproduce forever with
# nothing to catch it.
#
# Measured running it against the committed data (2026-08-08): CHOMP's own
# condition 2 is UNMET at its own r* (0.02, port 379/380 against C++'s own
# 370/370 there) -- STOMP's is MET (r* 0.05, port 441/441). Both were true
# before this gate existed; nothing had printed either one anywhere a reader
# would see it. This gate does not adjudicate whether CHOMP's UNMET is a
# porting defect (that reading belongs with whoever owns Phase 8's condition-2
# carve-out) -- it pins today's answer as a transcript so a future change to
# either side's data, or to this analysis script's own logic, is caught by
# name instead of staying silent.
#
# Re-running the sweeps themselves is not the fix: the STOMP arm alone costs
# hours and needs the oracle container (see `measure-phase8-condition2-
# grid.sh`'s and `measure-phase8-cpp-baseline.sh`'s own headers). The fix is
# that both sides' NDJSONs are already committed by `check-phase8-baseline-
# 500.sh`'s own directory, and re-deriving this script's report from them
# costs under a second, with no oracle, no docker, no cargo and no upstream
# checkout -- exactly what `.github/workflows/ci.yml`'s `check-*` glob
# provides.
#
#   tools/ci/check-phase8-condition2-grid.sh
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"

. "$REPO_ROOT/tools/ci/gate-lib.sh"

require_caller_tree "$REPO_ROOT"
cd "$REPO_ROOT"

DATA_DIR="doc/phase8-baseline-500"
SCRIPT="tools/ci/analyse-phase8-condition2-grid.py"

# planner:transcript pairs. A literal list, not a glob over `condition2-
# grid.*.txt`: a glob reports OK over whatever survives, and the failure this
# gate exists to prevent is a planner's transcript going missing without the
# gate noticing -- the same reasoning `check-phase8-baseline-500.sh`'s own
# named `WANT_*` arrays give for not globbing their file lists either.
PLANNERS=(chomp stomp)

WORKDIR="$(mktemp -d)"
trap 'rm -rf "$WORKDIR"' EXIT

fail=0
for planner in "${PLANNERS[@]}"; do
  transcript="$DATA_DIR/condition2-grid.$planner.txt"
  if ! git ls-files --deduplicate --error-unmatch "$transcript" >/dev/null 2>&1; then
    echo "FAIL $transcript is not tracked -- the evidence is not committed" >&2
    fail=1
    continue
  fi

  got="$WORKDIR/$planner.txt"
  # The script's own exit code is not read here: CHOMP's own committed
  # transcript ends UNMET and exits 1 by the script's own convention (see
  # header) -- what this gate checks is that today's run reproduces that
  # transcript byte for byte, not that it reports MET.
  python3 "$SCRIPT" --planner "$planner" \
    --config floor_wall \
      --port "$DATA_DIR/port.$planner.floor_wall.ndjson" \
      --cpp "$DATA_DIR/cpp.$planner.floor_wall.ndjson" \
      --seed "$DATA_DIR/seed.floor_wall.ndjson" \
    --config cage \
      --port "$DATA_DIR/port.$planner.cage.ndjson" \
      --cpp "$DATA_DIR/cpp.$planner.cage.ndjson" \
      --seed "$DATA_DIR/seed.cage.ndjson" \
    >"$got" 2>&1 || true

  if ! diff -u "$transcript" "$got" >"$WORKDIR/$planner.diff"; then
    echo "FAIL $planner: today's report does not match $transcript" >&2
    cat "$WORKDIR/$planner.diff" >&2
    fail=1
  else
    echo "ok   $planner: reproduces $transcript"
  fi
done

if [ "$fail" -ne 0 ]; then
  echo "FAIL condition-2 grid analysis drifted from its committed transcript(s) --" >&2
  echo "  see the diff(s) above. A change to the committed NDJSONs or to" >&2
  echo "  analyse-phase8-condition2-grid.py's own logic moved the report; if the" >&2
  echo "  new report is correct, re-commit it as the new transcript deliberately" >&2
  echo "  rather than letting this gate paper over it." >&2
  exit 1
fi

echo "OK condition-2 grid analysis reproduces both planners' committed transcripts"
