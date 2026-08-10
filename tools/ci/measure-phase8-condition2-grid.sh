#!/bin/bash
# Runs this port's CHOMP or STOMP over one Phase 8 benchmark config with a
# condition-2 *operating-point grid* attached, and writes the per-problem
# results as NDJSON.
#
# This is the port-side twin of `measure-phase8-cpp-baseline.sh`: same CLI
# shape, same problem set, same per-problem seed rule, and it injects the same
# `condition2_resolutions` field into the same request the C++ side gets. The
# two NDJSONs are then comparable problem by problem at every resolution in
# the grid, which is what `analyse-phase8-condition2-grid.py` consumes.
#
# # Why a grid at all
#
# PORTING-PLAN.md's Phase 8 row leaves condition 2 unspecified for the
# CHOMP/STOMP class because upstream's own C++ implementations do not reach
# 100% at the Phase 7 resolution (`motion_resolution` = 0.01) either, so a
# miss there cannot separate a port defect from the class's own behaviour.
# Specifying it needs a resolution at which the C++ implementation of the
# SAME planner does reach 100%, and finding that resolution needs the verdict
# at several -- which is what this grid produces.
#
# The plan does not depend on the densification resolution on either side
# (both read it only after the planner returned, to densify the path it
# produced), so one sweep yields every resolution's verdict. That is not a
# convenience: a per-resolution re-run of the STOMP side would cost about
# three hours each and the measurement would not exist.
#
# # Cost (this machine, 96 cores)
#
# The grid's cost is the sum of its densified waypoint counts, and it is
# dominated by its finest entry -- 0.001 densifies a typical `cage` CHOMP
# path to ~2,300 states against 0.01's ~285. Measured here: the seven-point
# grid `0.01..0.0001` cost 8m22s for ten CHOMP problems in one process, i.e.
# about 47s of checking per problem, three quarters of it in the two entries
# below 0.001. The grid this round uses stops at 0.001 for that reason.
#
# CHOMP takes `SHARDS` processes (its per-problem cost is bounded by
# `max_iterations` = 50); STOMP takes one process per problem, because a
# problem that never reaches a valid trajectory runs all 1000 iterations and
# a shard containing one is bounded below by it. Both rules are
# `verify-phase8-benchmark.sh`'s, unchanged.
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"

. "$REPO_ROOT/tools/ci/gate-lib.sh"

require_caller_tree "$REPO_ROOT"

usage() {
  cat >&2 <<'USAGE'
usage: measure-phase8-condition2-grid.sh <chomp|stomp> <config> <count> <set_seed> <out_dir>

  config    obstacle configuration name (floor_wall, cage, ...)
  count     number of problems
  set_seed  seed the problem set itself is generated at (900001 / 900002)
  out_dir   created if absent; results land in
            <out_dir>/port.<planner>.<config>.ndjson

environment:
  CONDITION2_RESOLUTIONS  required, comma-separated densification resolutions
  PORT_SEED_BASE          per-problem planner RNG seed base (default 700001)
  SHARDS                  CHOMP-only process count (default 25)
USAGE
  exit 2
}

[ $# -eq 5 ] || usage
PLANNER="$1"
CONFIG="$2"
COUNT="$3"
SET_SEED="$4"
OUT_DIR="$5"

case "$PLANNER" in
  chomp | stomp) ;;
  *) usage ;;
esac

# Required, not defaulted: this script exists to produce the grid, and a run
# that silently produced only `motion_resolution`'s verdict would be reported
# under the grid's name by whatever reads its output.
if [ -z "${CONDITION2_RESOLUTIONS:-}" ]; then
  echo "FAIL CONDITION2_RESOLUTIONS is unset; this script measures nothing without it" >&2
  exit 2
fi
C2_GRID_JSON="[$CONDITION2_RESOLUTIONS]"

PORT_SEED_BASE="${PORT_SEED_BASE:-700001}"
SHARDS="${SHARDS:-25}"
# The port harnesses' non-binding clock argument, `verify-phase8-benchmark.sh`'s
# own value: large enough that each planner's iteration bound is what stops it.
NO_CLOCK_BOUND=1e9

SET_BIN="$REPO_ROOT/target/release/examples/plan_benchmark_problem_set"
case "$PLANNER" in
  chomp) PORT_BIN="$REPO_ROOT/target/release/examples/chomp_benchmark_port" ;;
  stomp) PORT_BIN="$REPO_ROOT/target/release/examples/stomp_benchmark_port" ;;
esac

if [ ! -x "$SET_BIN" ] || [ ! -x "$PORT_BIN" ]; then
  echo "building the example binaries (release)..." >&2
  cargo build --release \
    --example plan_benchmark_problem_set -p cspace-planners \
    --example chomp_benchmark_port -p cspace-planners \
    --example stomp_benchmark_port -p cspace-planners \
    --manifest-path "$REPO_ROOT/Cargo.toml" >&2
fi

mkdir -p "$OUT_DIR"
# Same filename the C++ side's set lands under, so a run of both scripts into
# one directory provably shares the problem set rather than generating two.
SET_JSON="$OUT_DIR/$CONFIG.$COUNT.$SET_SEED.set.json"
if [ ! -s "$SET_JSON" ]; then
  "$SET_BIN" "$CONFIG" "$COUNT" "$SET_SEED" >"$SET_JSON" 2>"$OUT_DIR/$CONFIG.stats"
fi

IN_JSON="$OUT_DIR/$CONFIG.$PLANNER.grid.in.json"
jq -c --argjson c2 "$C2_GRID_JSON" '.condition2_resolutions=$c2' "$SET_JSON" >"$IN_JSON"

N=$(jq '.problems | length' "$IN_JSON")
if [ "$N" -ne "$COUNT" ]; then
  echo "FAIL $CONFIG carries $N problems, not the $COUNT asked for" >&2
  exit 1
fi

# STOMP gets one process per problem; CHOMP gets $SHARDS. See the header.
if [ "$PLANNER" = stomp ]; then
  PROCS="$N"
else
  PROCS="$SHARDS"
fi

WORK="$OUT_DIR/work.$PLANNER.$CONFIG"
rm -rf "$WORK"
mkdir -p "$WORK"

PER=$(((N + PROCS - 1) / PROCS))
pids=()
for ((s = 0; s * PER < N; s++)); do
  lo=$((s * PER))
  hi=$((lo + PER))
  jq --argjson lo "$lo" --argjson hi "$hi" '.problems |= .[$lo:$hi]' "$IN_JSON" \
    >"$WORK/$s.in"
  "$PORT_BIN" "$PORT_SEED_BASE" "$NO_CLOCK_BOUND" \
    <"$WORK/$s.in" >"$WORK/$s.out" 2>"$WORK/$s.err" &
  pids+=($!)
done
echo "=== port $PLANNER / $CONFIG: $N problems over ${#pids[@]} processes, seed base $PORT_SEED_BASE ===" >&2
echo "=== grid $C2_GRID_JSON ===" >&2

# Every shard's status is checked: a harness that panicked would otherwise
# contribute an empty file and be reported as a smaller population.
failed=0
for pid in "${pids[@]}"; do
  wait "$pid" || failed=$((failed + 1))
done
if [ "$failed" -ne 0 ]; then
  echo "FAIL $failed of ${#pids[@]} $PLANNER/$CONFIG processes exited nonzero; see $WORK/*.err" >&2
  exit 1
fi

OUT="$OUT_DIR/port.$PLANNER.$CONFIG.ndjson"
cat "$WORK"/*.out | jq -s -c 'sort_by(.id) | .[]' >"$OUT"
got=$(grep -c . "$OUT")
if [ "$got" -ne "$N" ]; then
  echo "FAIL $PLANNER/$CONFIG produced $got verdicts for $N problems" >&2
  exit 1
fi

# A record that solved but carries no grid is the failure mode this script's
# required env var exists to prevent, caught again on the way out in case the
# binary predates the field.
missing=$(jq -s '[.[] | select(.solved) | select(has("condition2_by_resolution") | not)] | length' "$OUT")
if [ "$missing" -ne 0 ]; then
  echo "FAIL $missing solved records carry no condition2_by_resolution" >&2
  exit 1
fi

solved=$(jq -s '[.[] | select(.solved)] | length' "$OUT")
echo "port $PLANNER/$CONFIG: solved $solved/$N -> $OUT" >&2
