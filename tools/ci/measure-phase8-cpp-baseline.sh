#!/bin/bash
# Runs upstream's own C++ CHOMP or STOMP over one Phase 8 benchmark config,
# one oracle process per problem, and writes the per-problem results as NDJSON.
#
# This is the C++ side of the comparison PORTING-PLAN.md's Phase 8 section
# needs. The port side is `chomp_benchmark_port.rs` / `stomp_benchmark_port.rs`;
# both consume the same request emitted by `plan_benchmark_problem_set`, so the
# two sides see byte-identical problems and differ only in which
# implementation solves them.
#
# # Why one process per problem
#
# Both C++ planners draw every random number from `rsl::rng()`, a
# `thread_local std::mt19937` that can be seeded exactly once per thread --
# `rsl::rng` throws on any later call carrying a seed sequence. So a seed is a
# property of a process, not of a request, and the oracle takes it as the
# startup argument `--planner-rng-seed` (see `oracle.cpp`'s `main`, which
# applies it before any `RobotModel` exists because `moveit::getLogger()`
# otherwise takes that one seedable call for itself).
#
# Running a whole config in one process would therefore make every problem's
# result depend on the entire stream of problems before it: reproducible only
# as a whole, and silently changed by any sharding. One process per problem at
# `--planner-rng-seed $((PLANNER_SEED_BASE + id))` instead mirrors the port
# harnesses exactly (both seed `seed_base.wrapping_add(problem.id)` per
# problem), which is what makes a per-problem comparison between the two sides
# meaningful and what makes this script safe to shard.
#
# The seed matters -- this is measured, not assumed. On `cage` ids 0..9 at
# `max_iterations=50`, C++ CHOMP solves 7/10 at seed base 700001 and 6/10 at
# 424242, disagreeing on ids 6 and 8. A success rate from either side is one
# draw from a seed lottery, and the tables built on this script say so.
#
# # The clock
#
# Both planners' upstream defaults stop on a wall clock -- CHOMP's
# `planning_time_limit_` (6.0s from `ChompParameters`'s constructor, 10.0s from
# `CHOMPInterface::loadParams`) and STOMP's `allowed_planning_time` (5.0s from
# `move_group`) -- which makes a success rate measured at them a property of
# how loaded this machine was rather than of the planner. PORTING-PLAN.md's
# Phase 8 section records that trap being sprung on the port side.
#
# So the clock here is set far above any observed per-problem time and the
# iteration bound (CHOMP `max_iterations`, STOMP `num_iterations`, both left at
# upstream defaults) is what terminates. It is set to a finite value rather
# than infinity on purpose: an unbounded run cannot distinguish "still working"
# from "hung", and upstream reports a clock stop as its own error code
# (`TIMED_OUT` for STOMP). The summary below counts those codes, so a run that
# quietly became clock-bounded says so in its own output instead of being
# reported as a planner failure.
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"

. "$REPO_ROOT/tools/ci/gate-lib.sh"

require_caller_tree "$REPO_ROOT"

usage() {
  cat >&2 <<'USAGE'
usage: measure-phase8-cpp-baseline.sh <chomp|stomp> <config> <count> <set_seed> <out_dir>

  config    obstacle configuration name (floor_wall, cage, ...)
  count     number of problems
  set_seed  seed the problem set itself is generated at (900001 / 900002)
  out_dir   created if absent; results land in <out_dir>/<planner>.<config>.ndjson

environment:
  PLANNER_SEED_BASE  per-problem planner RNG seed base (default 700001, the
                     same base the port harnesses use)
  CHOMP_MAX_ITERATIONS   default 50, `ChompParameters`'s own constructor value
  STOMP_CLOCK_BOUND      wall-clock ceiling in seconds (default 3600)
  CHOMP_CLOCK_BOUND      wall-clock ceiling in seconds (default 3600)
  JOBS               parallel oracle processes (default 12)
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

PLANNER_SEED_BASE="${PLANNER_SEED_BASE:-700001}"
CHOMP_MAX_ITERATIONS="${CHOMP_MAX_ITERATIONS:-50}"
CHOMP_CLOCK_BOUND="${CHOMP_CLOCK_BOUND:-3600}"
STOMP_CLOCK_BOUND="${STOMP_CLOCK_BOUND:-3600}"
JOBS="${JOBS:-12}"

URDF="$REPO_ROOT/fixtures/panda.urdf"
SRDF="$REPO_ROOT/fixtures/panda.srdf"
ORACLE="$REPO_ROOT/tools/moveit-oracle/run-oracle.sh"
BIN="$REPO_ROOT/target/release/examples/plan_benchmark_problem_set"

mkdir -p "$OUT_DIR"
SET_JSON="$OUT_DIR/$CONFIG.$COUNT.$SET_SEED.set.json"
REQ_DIR="$OUT_DIR/req.$PLANNER.$CONFIG"
RES_DIR="$OUT_DIR/res.$PLANNER.$CONFIG"
mkdir -p "$REQ_DIR" "$RES_DIR"

if [ ! -x "$BIN" ]; then
  echo "building examples/plan_benchmark_problem_set (release)..." >&2
  cargo build --release --example plan_benchmark_problem_set -p moveit-planners-sbp \
    --manifest-path "$REPO_ROOT/Cargo.toml" >&2
fi

# The problem set is generated once and reused for every problem's request, so
# every shard is provably looking at the same 500 problems as the port run and
# as the other planner.
if [ ! -s "$SET_JSON" ]; then
  "$BIN" "$CONFIG" "$COUNT" "$SET_SEED" >"$SET_JSON" 2>"$OUT_DIR/$CONFIG.stats"
fi

# One single-problem request per problem, carrying the whole object list and
# the set's own `motion_resolution` unchanged -- only `problems` is narrowed.
# `.op` is rewritten rather than regenerated so the geometry and the endpoints
# are literally the same bytes the `plan` op and the port harnesses consume.
if [ "$PLANNER" = chomp ]; then
  PLANNER_CFG=$(printf '{"max_iterations":%s,"planning_time_limit":%s}' \
    "$CHOMP_MAX_ITERATIONS" "$CHOMP_CLOCK_BOUND")
  jq -c --argjson cfg "$PLANNER_CFG" \
    '.op="chomp_plan" | .chomp=$cfg | . as $r
     | range(0; $r.problems|length) as $i
     | ($r | .problems=[$r.problems[$i]])' "$SET_JSON" \
    | while IFS= read -r line; do
        id=$(printf '%s' "$line" | jq -r '.problems[0].id')
        printf '%s\n' "$line" >"$REQ_DIR/$id.json"
      done
else
  PLANNER_CFG=$(printf '{"allowed_planning_time":%s}' "$STOMP_CLOCK_BOUND")
  jq -c --argjson cfg "$PLANNER_CFG" \
    '.op="stomp_plan" | .stomp=$cfg | . as $r
     | range(0; $r.problems|length) as $i
     | ($r | .problems=[$r.problems[$i]])' "$SET_JSON" \
    | while IFS= read -r line; do
        id=$(printf '%s' "$line" | jq -r '.problems[0].id')
        printf '%s\n' "$line" >"$REQ_DIR/$id.json"
      done
fi

solve_one() {
  local id="$1"
  local seed=$((PLANNER_SEED_BASE + id))
  local out="$RES_DIR/$id.json"
  [ -s "$out" ] && return 0
  local started
  started=$(date +%s.%N)
  # No pipe into jq here: a pipeline's status is its last stage's, so an oracle
  # that died would be recorded as a well-formed empty result.
  if ! sg docker -c "$ORACLE --urdf $URDF --srdf $SRDF --planner-rng-seed $seed" \
       <"$REQ_DIR/$id.json" >"$out.raw" 2>"$out.err"; then
    echo "FAIL problem $id: oracle exited nonzero, see $out.err" >&2
    return 1
  fi
  local elapsed
  elapsed=$(echo "$(date +%s.%N) - $started" | bc)
  jq -c --argjson secs "$elapsed" --argjson seed "$seed" \
    '.result.problems[0] + {planner_rng_seed: $seed, wall_secs: $secs}' \
    <"$out.raw" >"$out"
  rm -f "$out.raw"
}
export -f solve_one
export RES_DIR REQ_DIR ORACLE URDF SRDF PLANNER_SEED_BASE

echo "=== $PLANNER / $CONFIG: $COUNT problems, $JOBS jobs, seed base $PLANNER_SEED_BASE ===" >&2
seq 0 $((COUNT - 1)) | xargs -P "$JOBS" -I{} bash -c 'solve_one {}'

OUT="$OUT_DIR/$PLANNER.$CONFIG.ndjson"
: >"$OUT"
for id in $(seq 0 $((COUNT - 1))); do
  if [ ! -s "$RES_DIR/$id.json" ]; then
    echo "FAIL problem $id produced no result" >&2
    exit 1
  fi
  cat "$RES_DIR/$id.json" >>"$OUT"
done

# The clock-bound check the header describes: any TIMED_OUT means the wall
# clock, not the iteration bound, decided that problem, and the whole rate is
# then partly a measurement of this machine.
timed_out=$(jq -s '[.[] | select(.failure == "TIMED_OUT")] | length' "$OUT")
solved=$(jq -s '[.[] | select(.solved)] | length' "$OUT")
echo "$PLANNER/$CONFIG: solved $solved/$COUNT, timed_out $timed_out -> $OUT" >&2
if [ "$timed_out" -ne 0 ]; then
  echo "FAIL $timed_out problems hit the wall-clock bound; this rate measures the machine" >&2
  exit 1
fi
