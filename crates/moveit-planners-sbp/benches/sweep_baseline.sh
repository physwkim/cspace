#!/bin/bash
# Phase 7 (PORTING-PLAN.md §5, §118) C++ baseline measurement.
#
# Two things, both live-oracle, both against panda_arm:
#
# 1. An obstacle-difficulty sweep: 20 endpoint-valid (start, goal) pairs per
#    obstacle configuration (see `examples/plan_benchmark_problem_set.rs`'s
#    `obstacles_for`), run through the oracle's `plan` op (OMPL RRTConnect
#    over a space built to match this crate's own `JointModelGroupSpace`
#    bit-for-bit -- see `tests/plan_space_parity.rs`). Prints solved/rate,
#    median path length among solved problems, median `ptc_evaluations`, and
#    the endpoint-filter's own bad_ep count/rate.
# 2. The actual Phase 7 benchmark set: 250 `floor_wall` + 250 `cage` pairs
#    (500 total) -- see this file's own "# Why floor_wall + cage" section
#    below for why those two and not the other four -- run the same way,
#    printed as one combined C++ baseline: solved/rate and median length.
#    This is the number Phase 7's completion conditions 1 and 3 compare a
#    future port measurement against; that port-side measurement is
#    deliberately not built here (see PORTING-PLAN.md's round notes: getting
#    this baseline wrong would invalidate every port number measured against
#    it, so it is measured and reported on its own first).
#
# # Why floor_wall + cage
#
# This crate builds its own obstacle geometry (six named configs; see
# `plan_benchmark_problem_set.rs`'s own doc comment for exact box
# dimensions) rather than reusing another panel's numbers unseen (measuring
# your own claim before trusting it applies here). Run at seed 784512+i
# (`i` = config index in `CONFIGS`, one oracle process per config -- `plan`'s
# own doc comment: OMPL's RNG is seeded at most once per process, so each
# config needs its own fresh process to be independently reproducible), this
# sweep measured (2026-08-04, this oracle stamp):
#
#   config       solved   rate    med_len   med_iters   bad_ep   bad_ep_rate
#   empty        20/20   100.0%    2.4481       1          11       35.5%
#   floor        20/20   100.0%    2.2172       1           6       23.1%
#   floor_wall   20/20   100.0%    3.0447       6          33       62.3%
#   slot         20/20   100.0%    2.6152       1          22       52.4%
#   corridor     20/20   100.0%    2.5293       1          30       60.0%
#   cage         20/20   100.0%    3.0921      68          65       76.5%
#
# Every config solves 100% at this budget (`max_iterations=2000`) -- success
# rate alone has no discriminative power here, at any obstacle tightness,
# which is a stronger version of the same conclusion PORTING-PLAN.md's own
# round notes record from a different, untransferable obstacle geometry:
# the real gate is path length/iteration count, not success rate.
#
# `floor_wall` and `cage` are the only two configs whose median
# `ptc_evaluations` rises above the RRTConnect single-iteration floor (6 and
# 68 respectively, against 1 everywhere else) -- i.e. the only two where
# `connect`'s greedy growth is ever actually blocked rather than closing the
# gap in one step. `empty`/`floor`/`slot`/`corridor` measure almost nothing
# at this geometry and budget. This is *not* the same band another panel's
# untransferable-geometry measurement named ("floor_wall ~ corridor"): that
# band was measured against different obstacle boxes this crate has no way
# to reproduce, and re-measuring against this crate's own geometry is the
# point (PORTING-PLAN.md's own "don't cite a number you have not
# reproduced" rule applies to obstacle-difficulty bands too, not just to
# specific pass/fail counts). Nor does this measurement confirm the other
# panel's "tighter obstacles paradoxically get easier" finding: here `cage`
# (the tightest configuration, 76.5% bad_ep) is the *hardest* to plan
# through once a valid pair is found (highest median iterations of all six),
# not the easiest -- a different, still-measured, still-honestly-reported
# answer to the same "don't assume tighter is harder, measure it" question.
#
# The 500-problem set is therefore built from exactly the two configs this
# measurement shows real difficulty in, split evenly (250 each) rather than
# diluted by four configs that measure nothing.
#
# # Reproducing
#
# `sg docker -c '...'`, absolute paths only (relative paths fail inside the
# oracle container) -- run from anywhere, this script resolves its own repo
# root.

set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
URDF="$REPO_ROOT/fixtures/panda.urdf"
SRDF="$REPO_ROOT/fixtures/panda.srdf"
ORACLE="$REPO_ROOT/tools/moveit-oracle/run-oracle.sh"

WORKDIR="$(mktemp -d)"
trap 'rm -rf "$WORKDIR"' EXIT

echo "building examples/plan_benchmark_problem_set (release)..." >&2
cargo build --release --example plan_benchmark_problem_set -p moveit-planners-sbp \
  --manifest-path "$REPO_ROOT/Cargo.toml" >&2
BIN="$REPO_ROOT/target/release/examples/plan_benchmark_problem_set"

median() {
  # $1: jq filter selecting the array of numbers to take the median of.
  jq "$1"' | sort | if length==0 then null
                    elif (length % 2)==1 then .[length/2|floor]
                    else (.[length/2 - 1] + .[length/2]) / 2
                    end'
}

run_config() {
  # $1: config name, $2: pair count, $3: seed. Writes request/response/stats
  # into $WORKDIR/$1.{json,response.json,stats}.
  local config="$1" count="$2" seed="$3"
  "$BIN" "$config" "$count" "$seed" \
    >"$WORKDIR/$config.json" 2>"$WORKDIR/$config.stats"
  # shellcheck disable=SC2002
  cat "$WORKDIR/$config.json" \
    | sg docker -c "$ORACLE --urdf $URDF --srdf $SRDF" \
    >"$WORKDIR/$config.response.json" 2>"$WORKDIR/$config.oracle.stderr"
}

echo "=== obstacle-difficulty sweep (20 pairs/config) ===" >&2
BASE_SEED=784512
CONFIGS=(empty floor floor_wall slot corridor cage)
printf "%-12s %10s %8s %10s %10s %8s %10s\n" \
  config solved rate med_len med_iters bad_ep bad_ep_rate
for i in "${!CONFIGS[@]}"; do
  config="${CONFIGS[$i]}"
  seed=$((BASE_SEED + i))
  run_config "$config" 20 "$seed"

  n=$(jq '.result.problems | length' "$WORKDIR/$config.response.json")
  solved=$(jq '[.result.problems[] | select(.exact==true)] | length' \
    "$WORKDIR/$config.response.json")
  medlen=$(median '[.result.problems[] | select(.exact==true) | .length]' \
    <"$WORKDIR/$config.response.json")
  mediters=$(median '[.result.problems[].ptc_evaluations]' \
    <"$WORKDIR/$config.response.json")
  bad_ep=$(grep -oP 'bad_ep=\K[0-9]+' "$WORKDIR/$config.stats")
  bad_ep_rate=$(grep -oP 'bad_ep_rate=\K[0-9.]+%' "$WORKDIR/$config.stats")
  rate=$(echo "scale=1; 100*$solved/$n" | bc)
  printf "%-12s %5d/%-4d %6s%% %10s %10s %8s %10s\n" \
    "$config" "$solved" "$n" "$rate" "$medlen" "$mediters" "$bad_ep" "$bad_ep_rate"
done

echo "" >&2
echo "=== Phase 7 benchmark set: 250 floor_wall + 250 cage (500 total) ===" >&2
run_config floor_wall 250 900001
run_config cage 250 900002

jq -s '[.[0].result.problems[], .[1].result.problems[]]' \
  "$WORKDIR/floor_wall.response.json" "$WORKDIR/cage.response.json" \
  >"$WORKDIR/combined.json"

n=$(jq 'length' "$WORKDIR/combined.json")
solved=$(jq '[.[] | select(.exact==true)] | length' "$WORKDIR/combined.json")
medlen=$(median '[.[] | select(.exact==true) | .length]' <"$WORKDIR/combined.json")
mediters=$(median '[.[].ptc_evaluations]' <"$WORKDIR/combined.json")
rate=$(echo "scale=1; 100*$solved/$n" | bc)
printf "combined-500 solved=%d/%d rate=%s%% med_len=%s med_iters=%s\n" \
  "$solved" "$n" "$rate" "$medlen" "$mediters"
