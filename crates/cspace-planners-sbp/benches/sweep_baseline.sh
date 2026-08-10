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
# # n=20 was not enough to select on -- validated (and corrected) at n=250
#
# The table above is the *selection basis* for floor_wall+cage, not a
# claim that survived unchanged: n=20 is one sample, and choosing a
# benchmark config from a single small sample's median is the same shape of
# mistake as drawing a population conclusion from n=1, just at n=20 instead
# (PORTING-PLAN.md's own round notes record the n=1 version of this same
# mistake). The fix is validating the selection criterion at the scale it
# will actually be used at -- so before trusting the 500-problem set below,
# both configs were re-measured at the real n=250, same seeds (900001,
# 900002) as the final set:
#
#   config       n     solved   med_iters   iters_p75   iters_p90   iters_max   med_len
#   floor_wall   250   250/250      1          214         779        1553       2.5748
#   cage         250   248/250     31          249         548        2001       2.7537
#
# `floor_wall`'s n=20 signal (median 6) did **not** survive: at n=250 the
# median iteration count is 1, indistinguishable from the five configs this
# sweep already showed measure nothing. `cage`'s signal did survive (68 ->
# 31 -- still well above the single-iteration floor). Trusting the n=20
# table's `floor_wall` row unchanged would have been citing an unreproduced
# number; this is that check, done, with the result recorded rather than
# quietly substituted in above.
#
# This does **not** mean floor_wall should be dropped from the set. Look at
# the quantiles, not just the median: floor_wall's `iters_p75` is 214,
# `iters_p90` is 779, `iters_max` is 1553 -- half the problems are trivial
# one-iteration straight-line connections (pulling the median down to 1),
# but the other half contains real difficulty the median simply does not
# see. This directly threatens Phase 7 completion condition 3 ("median path
# length within 1.3x of C++"): if half of a config's problems resolve to
# near-identical length regardless of implementation quality, the median
# ratio can pass without that config's harder half ever being measured. §5
# still specifies the median as the official condition-3 statistic and
# `run_config`'s combined-set output below still reports it unchanged; the
# `quantile_report` block added below reports p50/p75/p90/max for both
# length and iteration count *alongside* it, on the full set and on a
# defined hard subset, specifically so a config like this cannot look
# uniformly easy just because its easy half outnumbers its hard half.
#
# # The hard subset, and the bias it deliberately introduces
#
# "Hard" is defined as: C++ `ptc_evaluations` >= 227 -- the combined
# 500-problem set's own p75 for iteration count (see the quantile report
# below; 227 is not invented for this purpose, it is the same quartile
# boundary already being reported). At this threshold: 125/500 problems
# (60 from floor_wall, 65 from cage) -- a quarter of the full set, evenly
# drawn from both configs, none of it the trivial one-iteration mode the
# median above is dominated by.
#
# Naming the bias this creates, not just the subset: selecting "hard" by
# C++'s own iteration count selects specifically for problems that were
# hard *for OMPL RRTConnect's particular tree-growth and RNG sequence* --
# not necessarily hard in an implementation-neutral, purely-geometric
# sense. A problem where C++ happened to grow its tree the wrong direction
# early (RNG-unlucky, not geometrically hard) lands in this subset the same
# as a problem behind a genuinely narrow passage; a problem that is
# genuinely narrow but where C++'s sampling sequence got lucky would be
# excluded. This is why the hard subset is used here as a *supplementary*
# diagnostic only, never as a substitute for Phase 7's official full-500
# gate: a port that does badly only on this subset might be failing on
# C++'s own weak points rather than on anything the port itself does
# differently, and a port that does *well* on this subset is not thereby
# excused from the full-set gate either. Both full-set and hard-subset
# numbers are reported; only the full-set numbers are load-bearing for
# Phase 7's three completion conditions.
#
# # Reproducing
#
# `sg docker -c '...'`, absolute paths only (relative paths fail inside the
# oracle container) -- run from anywhere, this script resolves its own repo
# root.

set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"

. "$REPO_ROOT/tools/ci/gate-lib.sh"

require_caller_tree "$REPO_ROOT"
URDF="$REPO_ROOT/fixtures/panda.urdf"
SRDF="$REPO_ROOT/fixtures/panda.srdf"
ORACLE="$REPO_ROOT/tools/moveit-oracle/run-oracle.sh"

WORKDIR="$(mktemp -d)"
trap 'rm -rf "$WORKDIR"' EXIT

echo "building examples/plan_benchmark_problem_set (release)..." >&2
cargo build --release --example plan_benchmark_problem_set -p cspace-planners-sbp \
  --manifest-path "$REPO_ROOT/Cargo.toml" >&2
BIN="$REPO_ROOT/target/release/examples/plan_benchmark_problem_set"

median() {
  # $1: jq filter selecting the array of numbers to take the median of.
  jq "$1"' | sort | if length==0 then null
                    elif (length % 2)==1 then .[length/2|floor]
                    else (.[length/2 - 1] + .[length/2]) / 2
                    end'
}

# p50/p75/p90/max for path length (solved problems only) and iteration
# count (every problem, matching `median()`'s own `.ptc_evaluations`
# selection above), over whatever array of oracle `problems[]` entries is
# piped in on stdin. p50 uses the same average-of-two-middle convention as
# `median()` (must match it exactly -- computing p50 by nearest-rank here
# and by averaged-middle in `median()` would silently disagree on an
# even-length array and look like a data discrepancy rather than the
# methodology mismatch it would actually be). p75/p90 use nearest-rank
# (`sort | .[(length*p)|floor]`); ties in `length*p` landing on an exact
# grid line are a nearest-rank/rank-averaging judgment call this script
# does not need to resolve any more finely than that, since it is reported
# as a diagnostic quantile, not compared against a threshold at that
# precision.
quantile_report() {
  # $1: label to prefix the printed line with. Reads a JSON array of
  # `problems[]` entries on stdin.
  local label="$1"
  jq --arg label "$label" '
    def median:
      sort
      | if length == 0 then null
        elif (length % 2) == 1 then .[length/2 | floor]
        else (.[length/2 - 1] + .[length/2]) / 2
        end;
    def q(p): sort | .[(length * p) | floor];
    {
      label: $label,
      n: length,
      solved: ([.[] | select(.exact == true)] | length),
      len_p50: ([.[] | select(.exact == true) | .length] | median),
      len_p75: ([.[] | select(.exact == true) | .length] | q(0.75)),
      len_p90: ([.[] | select(.exact == true) | .length] | q(0.90)),
      len_max: ([.[] | select(.exact == true) | .length] | max),
      iters_p50: ([.[] | .ptc_evaluations] | median),
      iters_p75: ([.[] | .ptc_evaluations] | q(0.75)),
      iters_p90: ([.[] | .ptc_evaluations] | q(0.90)),
      iters_max: ([.[] | .ptc_evaluations] | max)
    }
    | "\(.label) n=\(.n) solved=\(.solved) len_p50=\(.len_p50) len_p75=\(.len_p75) len_p90=\(.len_p90) len_max=\(.len_max) iters_p50=\(.iters_p50) iters_p75=\(.iters_p75) iters_p90=\(.iters_p90) iters_max=\(.iters_max)"
  ' -r
}

# Combined-set p75 of `ptc_evaluations` (see "# The hard subset" above for
# why this exact threshold and not another) -- computed once from the
# combined 500-problem set so it is never out of sync with what
# `quantile_report`'s own `iters_p75` reports for that set.
HARD_SUBSET_K=227

run_config() {
  # $1: config name, $2: pair count, $3: seed. Writes request/response/stats
  # into $WORKDIR/$1.{json,response.json,stats}.
  local config="$1" count="$2" seed="$3"
  "$BIN" "$config" "$count" "$seed" \
    >"$WORKDIR/$config.json" 2>"$WORKDIR/$config.stats"
  sg docker -c "$ORACLE --urdf $URDF --srdf $SRDF" \
    <"$WORKDIR/$config.json" \
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

echo "" >&2
echo "=== quantiles (§5 condition 3 stays median-only; this is diagnostic) ===" >&2
jq '.result.problems' "$WORKDIR/floor_wall.response.json" | quantile_report "floor_wall"
jq '.result.problems' "$WORKDIR/cage.response.json" | quantile_report "cage"
quantile_report "combined-500" <"$WORKDIR/combined.json"

jq --argjson k "$HARD_SUBSET_K" '[.[] | select(.ptc_evaluations >= $k)]' \
  "$WORKDIR/combined.json" >"$WORKDIR/hard_subset.json"
echo "" >&2
echo "=== hard subset: C++ ptc_evaluations >= $HARD_SUBSET_K (see doc for the selection bias this introduces) ===" >&2
quantile_report "hard-subset" <"$WORKDIR/hard_subset.json"
