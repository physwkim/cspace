#!/bin/bash
# Runs this port's CHOMP over the Phase 8 benchmark's 500 problems and reports
# what its own objective function did on each one: the cost of the seed
# trajectory it was handed, the cost of the trajectory it returned, and the
# cost of the last iterate its loop evaluated.
#
# # Why this is a separate script and not a line in verify-phase8-benchmark.sh
#
# `verify-phase8-benchmark.sh` costs hours: it runs the C++ OMPL oracle over
# both configs and then STOMP at one process per problem, whose slowest
# problem alone dominates the wall clock. The CHOMP half of that gate is the
# cheap part and it is the only part this measurement reads, so this script
# runs exactly that half -- same configs, same counts, same set seeds, same
# `PORT_SEED_BASE`, same non-binding clock argument, same shard rule -- and
# skips the oracle and STOMP entirely. Its output is therefore the same
# CHOMP NDJSON that gate produces, with the `objective` field read out of it.
#
# # Why the numbers are reproducible and the wall clock is not
#
# `NO_CLOCK_BOUND` is the gate's own `1e9`: with it, nothing in a CHOMP run
# depends on how fast the machine ran it, so `solved` and every objective
# number below are exact re-derivations and are pinned as equalities. The
# wall clock this script prints is not -- it is a machine-and-load reading and
# is labelled as one.
#
# # Cost (this machine, 96 cores, 25 shards)
#
# About one minute for the 500 problems, plus ~15s to generate the two
# problem sets. That is cheap enough to need no opt-in gating; the expensive
# thing this script deliberately does not do is the oracle and STOMP halves
# of the gate it is carved out of.
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"

. "$REPO_ROOT/tools/ci/gate-lib.sh"

require_caller_tree "$REPO_ROOT"

usage() {
  cat >&2 <<'USAGE'
usage: measure-chomp-objective.sh <out_dir>

  out_dir   created if absent; per-config NDJSON and the summary land here

environment:
  PORT_SEED_BASE  per-problem planner RNG seed base (default 700001,
                  verify-phase8-benchmark.sh's own value)
  SHARDS          process count per config (default 25, the gate's value)
USAGE
  exit 2
}

[ $# -eq 1 ] || usage
OUT_DIR="$1"

# verify-phase8-benchmark.sh's CONFIGS / COUNTS / SEEDS / PORT_SEED_BASE /
# NO_CLOCK_BOUND, unchanged. Changing any of them measures a different
# population than the one this port's pinned CHOMP numbers came from.
CONFIGS=(floor_wall cage)
COUNTS=(250 250)
SET_SEEDS=(900001 900002)
PORT_SEED_BASE="${PORT_SEED_BASE:-700001}"
SHARDS="${SHARDS:-25}"
NO_CLOCK_BOUND=1e9

SET_BIN="$REPO_ROOT/target/release/examples/plan_benchmark_problem_set"
PORT_BIN="$REPO_ROOT/target/release/examples/chomp_benchmark_port"

echo "building the example binaries (release)..." >&2
cargo build --release \
  --example plan_benchmark_problem_set -p cspace-planners \
  --example chomp_benchmark_port -p cspace-planners \
  --manifest-path "$REPO_ROOT/Cargo.toml" >&2

mkdir -p "$OUT_DIR"

started=$(date +%s)
ALL="$OUT_DIR/chomp.all.ndjson"
: >"$ALL"

for i in "${!CONFIGS[@]}"; do
  config="${CONFIGS[$i]}"
  count="${COUNTS[$i]}"
  set_seed="${SET_SEEDS[$i]}"

  set_json="$OUT_DIR/$config.$count.$set_seed.set.json"
  if [ ! -s "$set_json" ]; then
    "$SET_BIN" "$config" "$count" "$set_seed" \
      >"$set_json" 2>"$OUT_DIR/$config.stats"
  fi
  n=$(jq '.problems | length' "$set_json")
  if [ "$n" -ne "$count" ]; then
    echo "FAIL $config carries $n problems, not the $count asked for" >&2
    exit 1
  fi

  work="$OUT_DIR/work.$config"
  rm -rf "$work"
  mkdir -p "$work"

  per=$(((n + SHARDS - 1) / SHARDS))
  pids=()
  for ((s = 0; s * per < n; s++)); do
    lo=$((s * per))
    hi=$((lo + per))
    jq --argjson lo "$lo" --argjson hi "$hi" '.problems |= .[$lo:$hi]' "$set_json" \
      >"$work/$s.in"
    "$PORT_BIN" "$PORT_SEED_BASE" "$NO_CLOCK_BOUND" \
      <"$work/$s.in" >"$work/$s.out" 2>"$work/$s.err" &
    pids+=($!)
  done
  echo "=== chomp / $config: $n problems over ${#pids[@]} processes, seed base $PORT_SEED_BASE ===" >&2

  # Every shard's status is checked: a shard that panicked would otherwise
  # contribute an empty file and be reported as a smaller population.
  failed=0
  for pid in "${pids[@]}"; do
    wait "$pid" || failed=$((failed + 1))
  done
  if [ "$failed" -ne 0 ]; then
    echo "FAIL $failed of ${#pids[@]} chomp/$config processes exited nonzero; see $work/*.err" >&2
    exit 1
  fi

  out="$OUT_DIR/chomp.$config.ndjson"
  cat "$work"/*.out | jq -s -c 'sort_by(.id) | .[]' >"$out"
  got=$(grep -c . "$out")
  if [ "$got" -ne "$n" ]; then
    echo "FAIL chomp/$config produced $got verdicts for $n problems" >&2
    exit 1
  fi
  # A solved record with no `objective` is what this whole script measures the
  # absence of; catching it here rather than in the summary keeps a binary
  # that predates the field from being reported as "0 problems made worse".
  missing=$(jq -s '[.[] | select(.solved) | select(has("objective") | not)] | length' "$out")
  if [ "$missing" -ne 0 ]; then
    echo "FAIL $missing solved chomp/$config records carry no objective" >&2
    exit 1
  fi
  # Same rule for `loop`: the split below is a statement about the loop, so a
  # binary that predates the field must not be summarised as "0 problems left
  # before evaluating an update".
  missing=$(jq -s '[.[] | select(.solved) | select(has("loop") | not)] | length' "$out")
  if [ "$missing" -ne 0 ]; then
    echo "FAIL $missing solved chomp/$config records carry no loop trace" >&2
    exit 1
  fi
  # Tag each record with its config before merging, so the summary can split
  # by config without a second read of the per-config files.
  jq -c --arg config "$config" '. + {config: $config}' "$out" >>"$ALL"
done

run_seconds=$(($(date +%s) - started))

SUMMARY="$OUT_DIR/chomp.objective.summary.json"
jq -s '
  def stats(xs): (xs | length) as $n
    | if $n == 0 then {n: 0}
      else (xs | sort) as $s
        | {n: $n, min: $s[0], max: $s[$n - 1],
           median: (if $n % 2 == 1 then $s[($n - 1) / 2]
                    else ($s[$n / 2 - 1] + $s[$n / 2]) / 2 end),
           mean: ((xs | add) / $n)}
      end;
  # The loop trace, read over an arbitrary subset of solved records. The point
  # of every field here is to distinguish "the loop ran and never lowered the
  # cost" from "the loop never costed a second iterate at all": `evaluations`
  # is 1 exactly when the run left before any updated trajectory was evaluated,
  # and in that case no explanation that needs a second evaluation can hold.
  def loopstats(rows): (rows | map(.loop)) as $l
    | {n: (rows | length),
       evaluations: stats($l | map(.evaluations)),
       evaluations_is_1: ($l | map(select(.evaluations == 1)) | length),
       exit: ($l | group_by(.exit)
                 | map({key: .[0].exit, value: length}) | from_entries),
       accepted: stats($l | map(.accepted)),
       accepted_is_1: ($l | map(select(.accepted == 1)) | length),
       mesh_free_passes: stats($l | map(.mesh_free_passes)),
       below_threshold_passes: stats($l | map(.below_threshold_passes)),
       below_threshold_ever: ($l | map(select(.below_threshold_passes > 0)) | length),
       seed_points_within_clearance: stats($l | map(.seed_points_within_clearance)),
       seed_points_in_collision: stats($l | map(.seed_points_in_collision)),
       seed_points_in_collision_is_0:
         ($l | map(select(.seed_points_in_collision == 0)) | length),
       first_pass_max_update: stats($l | map(.first_pass_max_update)),
       seed_total: stats(rows | map(.objective.seed.total)),
       seed_collision: stats(rows | map(.objective.seed.collision)),
       seed_smoothness: stats(rows | map(.objective.seed.smoothness))};
  def summarise(rows):
    (rows | map(select(.solved))) as $solved
    | ($solved | map(.objective)) as $o
    | {problems: (rows | length),
       solved: ($solved | length),
       # `best` is the objective of the trajectory that was returned;
       # `improvement` cannot be negative by construction, so this count
       # being 0 measures the best-snapshot, not the optimizer.
       returned_worse_than_seed: ($o | map(select(.improvement < 0)) | length),
       returned_equal_to_seed: ($o | map(select(.improvement == 0)) | length),
       returned_better_than_seed: ($o | map(select(.improvement > 0)) | length),
       # `last` is the final iterate the loop evaluated and then discarded.
       # This count is the one whose sign is open.
       final_iterate_worse_than_seed: ($o | map(select(.descent < 0)) | length),
       final_iterate_is_the_returned_one: ($o | map(select(.descent == .improvement)) | length),
       seed_total: stats($o | map(.seed.total)),
       best_total: stats($o | map(.best.total)),
       last_total: stats($o | map(.last.total)),
       improvement: stats($o | map(.improvement)),
       descent: stats($o | map(.descent)),
       relative_improvement: stats($o | map(select(.seed.total > 0)
                                            | .improvement / .seed.total)),
       # Which of the two terms the improvement came out of.
       smoothness_improvement: stats($o | map(.seed.smoothness - .best.smoothness)),
       collision_improvement: stats($o | map(.seed.collision - .best.collision)),
       seed_collision_is_zero: ($o | map(select(.seed.collision == 0)) | length),
       # The same solved rows, split by the stratum the round is about: the
       # problems the optimizer returned the seed on, and the ones it beat it
       # on. Pooling the two hides whichever fact separates them.
       loop: {all: loopstats($solved),
              improvement_zero:
                loopstats($solved | map(select(.objective.improvement == 0))),
              improvement_positive:
                loopstats($solved | map(select(.objective.improvement > 0)))}};
  {all: summarise(.),
   by_config: (group_by(.config) | map({key: .[0].config, value: summarise(.)})
               | from_entries)}
' "$ALL" >"$SUMMARY"

echo >&2
echo "=== chomp objective over the Phase 8 500 (seed base $PORT_SEED_BASE, clock $NO_CLOCK_BOUND) ===" >&2
jq 'del(.all.loop, .by_config[].loop)' "$SUMMARY" >&2
echo >&2
# The discrimination table: one row per (config, improvement stratum). Printed
# separately from the JSON above because the question this round asks is a
# comparison between two strata within a config, and that comparison is
# unreadable as nested objects.
echo "=== loop trace, by config and improvement stratum ===" >&2
jq -r '
  def f(x): if x == null then "-" else (x | tostring | .[0:9]) end;
  def row($cfg; $stratum; $l):
    [$cfg, $stratum, ($l.n | tostring),
     (f($l.evaluations.min) + "/" + f($l.evaluations.median) + "/" + f($l.evaluations.max)),
     ($l.evaluations_is_1 | tostring),
     ($l.exit | to_entries | map(.key + "=" + (.value | tostring)) | join(",")),
     (f($l.accepted.min) + "/" + f($l.accepted.median) + "/" + f($l.accepted.max)),
     ($l.below_threshold_ever | tostring),
     (f($l.seed_points_within_clearance.min) + "/" + f($l.seed_points_within_clearance.median) + "/" + f($l.seed_points_within_clearance.max)),
     (f($l.seed_points_in_collision.min) + "/" + f($l.seed_points_in_collision.median) + "/" + f($l.seed_points_in_collision.max)),
     (f($l.first_pass_max_update.min) + "/" + f($l.first_pass_max_update.median) + "/" + f($l.first_pass_max_update.max)),
     (f($l.seed_total.median)), (f($l.seed_collision.median))]
    | @tsv;
  # `. as $root` must be bound before the header, because `|` binds looser than
  # `,`: without it the rows are generated with `.` already rewritten to the
  # header array.
  . as $root
  | ["config", "stratum", "n", "evals min/med/max", "evals==1", "exit",
     "accepted min/med/max", "belowthr>0", "clearpts min/med/max",
     "collpts min/med/max", "firstupd min/med/max", "seedtot med", "seedcoll med"]
  | @tsv,
  ( ($root.by_config | to_entries | map({cfg: .key, s: .value.loop}))
      + [{cfg: "ALL", s: $root.all.loop}]
    | .[] as $c
    | ("improvement==0", "improvement>0", "all") as $k
    | row($c.cfg;
          $k;
          (if $k == "improvement==0" then $c.s.improvement_zero
           elif $k == "improvement>0" then $c.s.improvement_positive
           else $c.s.all end)) )
' "$SUMMARY" | column -t -s$'\t' >&2
echo >&2
printf 'wall clock: %ss (a machine-and-load reading; every number above is not)\n' \
  "$run_seconds" >&2
echo "records: $ALL" >&2
echo "summary: $SUMMARY" >&2
