#!/bin/bash
# PORTING-PLAN.md §5 Phase 8's CHOMP/STOMP completion clause, as a command
# rather than a number in a report.
#
# §5 line 719 says the CHOMP/STOMP half of Phase 8 is verified by "Phase 7과
# 같은 속성 기반 검증" -- Phase 7's three property conditions (§5 lines
# 704-708), re-run with `moveit-planners-chomp` and `moveit-planners-stomp`
# in place of `moveit-planners-sbp` on the port side:
#
#   1. success rate >= 90% of C++ OMPL RRTConnect's, over 500 problems
#   2. 100% of returned paths pass `moveit-scene`'s collision + constraint check
#   3. median path length within 1.3x of C++ OMPL's
#
# # Which baseline, and why this reading
#
# "Phase 7과 같은" is read here as "the same three conditions against the
# same C++ OMPL RRTConnect baseline", not as "against a C++ CHOMP/STOMP".
# That WAS forced rather than chosen, and is no longer: the oracle now answers
# `chomp_plan` and `stomp_plan` (`oracle.cpp`'s `chompPlan`/`stompPlan`, over
# upstream `ChompPlanner::solve` and `StompPlanningContext::solve`), which
# `tools/ci/measure-phase8-cpp-baseline.sh` drives. This gate keeps the
# RRTConnect reading because §5 names that baseline by line, and
# `tools/ci/measure-phase8-optimizer-properties.sh` measures the
# planner-against-its-own-upstream reading alongside it -- two baselines
# answering two different questions, neither replacing the other. The
# consequence of THIS one is stated rather than hidden -- conditions 1
# and 3 compare an optimiser (CHOMP/STOMP refine one seed trajectory) with a
# sampling planner (RRTConnect grows trees), so a miss is not by itself
# evidence of a porting defect. What the conditions do measure is exactly
# what §5 asks for: whether these two planners, as ported, clear the bar §5
# sets in terms of the baseline §5 names.
#
# # Why the bounds are set to non-binding values
#
# Both planners' upstream defaults contain a *wall-clock* stop --
# `ChompParameters::planning_time_limit` (6.0s) and STOMP's
# `allowed_planning_time` watchdog (5.0s, `MoveGroupInterface`'s default).
# Measured at those defaults the port-side success rate is not reproducible:
# two full 500-problem CHOMP sweeps at identical seeds on this machine gave
# 359/500 and then 349/500, disagreeing on 12 problems, purely with machine
# load; and every one of STOMP's 246 failures at 5.0s was the watchdog
# firing, i.e. the entire STOMP number was a measurement of this machine.
#
# So both harnesses take the time limit as an argument and this gate passes
# `1e9` -- large enough to never bind. What remains is each planner's own
# *iteration* bound, which is upstream's default and is deterministic:
# `ChompParameters::max_iterations` = 50 and STOMP's `num_iterations` = 1000
# (`res/stomp_moveit.yaml`). Every loop in the sweep is therefore bounded by
# an iteration count, not by a clock. The wall-clock numbers are reported in
# PORTING-PLAN.md §263 alongside these, as a separate, explicitly
# machine-dependent measurement.
#
# # Sharding
#
# Each problem's RNG is seeded from `seed_base + id`, the harnesses hold no
# state across problems, and with the clock bound non-binding there is no
# wall-clock input left -- so splitting the problems across N processes
# cannot change any per-problem verdict, only the wall clock. That claim was
# checked rather than asserted (PORTING-PLAN.md §263 records it): CHOMP's
# first 20 `floor_wall` problems run in one unsharded process are
# byte-identical to the same 20 lifted out of the 25-way sharded 500, and
# ids 0, 6 and 221 re-run one problem per process reproduce their sweep
# records exactly. Setting `PHASE8_SHARDS=1` re-checks it end to end at the
# cost of a serial CHOMP sweep.
#
# # Cost (measured on this machine, 96 cores, 2026-08-06)
#
#   C++ OMPL baseline (2 oracle processes)   see the "baseline wall=" line
#   CHOMP, 500 problems, 25 shards           170s and 143s on two runs
#   STOMP, 500 problems, one proc/problem    ~2h50m (86min floor_wall at 50
#                                            shards + 83min cage at 250)
#
# STOMP's cost is why this script has three tiers. A STOMP problem that
# never reaches a valid trajectory runs all 1000 iterations, and one such
# problem costs tens of CPU-minutes -- so the wall clock of *any* STOMP
# subset containing one is bounded below by that single problem, no matter
# how it is sharded. Measured here (`wall=` lines, this machine, 2026-08-06):
#
#   MOVEIT_RS_PHASE8_BENCHMARK=chomp  baseline + CHOMP's full 500     165s
#   MOVEIT_RS_PHASE8_BENCHMARK=1      + STOMP's pinned 50-problem     6629s
#                                       prefix subset
#   MOVEIT_RS_PHASE8_BENCHMARK=full   + STOMP's full 500              ~3h
#
# Every tier compares every number it derives against a pinned value and
# exits non-zero on any mismatch; the tiers differ only in how much of STOMP
# is re-derived, and each says on stdout what it did not cover. `chomp` is
# the one cheap enough for a routine merge round. Unset, the script SKIPs --
# loudly, because `verify-all.sh` counts a script that exits 0 after
# printing SKIP as passed.
#
#   MOVEIT_RS_PHASE8_BENCHMARK=chomp sg docker -c ./tools/ci/verify-phase8-benchmark.sh
#
# Absolute paths only for the oracle: `run-oracle.sh` mounts host paths at
# the same paths inside the container.
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"

. "$REPO_ROOT/tools/ci/gate-lib.sh"

require_caller_tree "$REPO_ROOT"
cd "$REPO_ROOT"

TIER="${MOVEIT_RS_PHASE8_BENCHMARK:-}"
if [[ -z "$TIER" ]]; then
  echo "SKIP MOVEIT_RS_PHASE8_BENCHMARK is unset -- Phase 8's CHOMP/STOMP property conditions are not re-derived."
  echo "SKIP this is not a pass; set MOVEIT_RS_PHASE8_BENCHMARK=chomp (~165s), =1 (adds STOMP's 50-problem prefix, ~110min) or =full (adds STOMP's 500, ~3h) to cover them."
  exit 0
fi
if [[ "$TIER" != "chomp" && "$TIER" != "1" && "$TIER" != "full" ]]; then
  echo "FAIL MOVEIT_RS_PHASE8_BENCHMARK=$TIER is not one of chomp, 1, full." >&2
  exit 1
fi

URDF="$REPO_ROOT/fixtures/panda.urdf"
SRDF="$REPO_ROOT/fixtures/panda.srdf"
ORACLE="$REPO_ROOT/tools/moveit-oracle/run-oracle.sh"

# No docker-reachability probe: an unreachable docker is a hard failure
# here, the same way it is in `verify-constraint-sweep.sh` and
# `verify-oracle-sweep.sh`. A second SKIP path would be a second way for
# this script to exit 0 having measured nothing, which is the exact failure
# the opt-in SKIP above already has to shout about.

# The Phase 7 benchmark set, unchanged: the same two configs, counts and
# seeds `crates/moveit-planners-sbp/benches/sweep_baseline.sh` measured the
# recorded C++ baseline from. Changing any of these invalidates every pinned
# number below.
CONFIGS=(floor_wall cage)
COUNTS=(250 250)
SEEDS=(900001 900002)
PORT_SEED_BASE=700001

# The port-side harnesses' non-binding clock argument. See the header.
NO_CLOCK_BOUND=1e9

# STOMP tier 1's subset: the first 25 problems of each config. A prefix, not
# a sample: it has to be reproducible from the same request JSON with no
# selection step of its own, and the ids are the sharding unit already.
STOMP_TIER1_PREFIX=25

SHARDS="${PHASE8_SHARDS:-25}"
if ! [[ "$SHARDS" =~ ^[0-9]+$ ]] || [[ "$SHARDS" -lt 1 ]]; then
  echo "FAIL PHASE8_SHARDS=$SHARDS is not a positive integer." >&2
  exit 1
fi

# Every number this gate re-derives, pinned. Exact equality, not a
# tolerance: the configuration above has no wall-clock input left, so a
# re-run that differs at all has changed something real. `verify-all.sh`
# reports the first mismatch with both values.
EXPECTED_CPP_SOLVED=498
EXPECTED_CPP_MEDIAN=2.6597767032746464
EXPECTED_CHOMP_SOLVED=380
EXPECTED_CHOMP_COND2=379
EXPECTED_CHOMP_MEDIAN=2.163978163668814
EXPECTED_STOMP_SOLVED=441
EXPECTED_STOMP_COND2=438
EXPECTED_STOMP_MEDIAN=2.210362452483207
EXPECTED_STOMP50_SOLVED=42
EXPECTED_STOMP50_COND2=42
EXPECTED_STOMP50_MEDIAN=2.107273368105653

WORKDIR="$(mktemp -d)"
trap 'rm -rf "$WORKDIR"' EXIT

started=$(date +%s)

echo "building the three example binaries (release)..." >&2
cargo build --release \
  --example plan_benchmark_problem_set -p moveit-planners-sbp \
  --example chomp_benchmark_port -p moveit-planners-chomp \
  --example stomp_benchmark_port -p moveit-planners-stomp >&2

SET_BIN="$REPO_ROOT/target/release/examples/plan_benchmark_problem_set"
CHOMP_BIN="$REPO_ROOT/target/release/examples/chomp_benchmark_port"
STOMP_BIN="$REPO_ROOT/target/release/examples/stomp_benchmark_port"

# --- the 500 problems, and the C++ OMPL baseline over them ----------------

t0=$(date +%s)
for i in "${!CONFIGS[@]}"; do
  c="${CONFIGS[$i]}"
  "$SET_BIN" "$c" "${COUNTS[$i]}" "${SEEDS[$i]}" \
    >"$WORKDIR/$c.json" 2>"$WORKDIR/$c.stats"
  # One oracle process per config: OMPL's RNG is seeded at most once per
  # process, so a shared process would make config 2 depend on config 1.
  sg docker -c "$ORACLE --urdf $URDF --srdf $SRDF" \
    <"$WORKDIR/$c.json" >"$WORKDIR/$c.response.json" 2>"$WORKDIR/$c.oracle.stderr"
done
echo "baseline wall=$(( $(date +%s) - t0 ))s" >&2

# --- port-side sweeps -----------------------------------------------------

# Runs one harness over one config's problems, split across $6 processes,
# and writes the verdicts back in id order. Bounded by the problem count:
# each shard reads a fixed slice and exits.
sweep() {
  local bin="$1" tag="$2" config="$3" src="$4" out="$5" shards="$6"
  local n per lo hi s
  local -a pids=()
  n=$(jq '.problems | length' "$src")
  per=$(( (n + shards - 1) / shards ))
  for (( s = 0; s < shards; s++ )); do
    lo=$(( s * per )); hi=$(( lo + per ))
    [[ "$lo" -ge "$n" ]] && break
    jq --argjson lo "$lo" --argjson hi "$hi" '.problems |= .[$lo:$hi]' "$src" \
      >"$WORKDIR/$config.$tag.$s.in"
    "$bin" "$PORT_SEED_BASE" "$NO_CLOCK_BOUND" \
      <"$WORKDIR/$config.$tag.$s.in" >"$WORKDIR/$config.$tag.$s.out" &
    pids+=($!)
  done
  for s in "${pids[@]}"; do wait "$s"; done
  cat "$WORKDIR/$config.$tag".*.out | jq -s -c 'sort_by(.id) | .[]' >"$out"
  local got
  got=$(grep -c . "$out")
  if [[ "$got" -ne "$n" ]]; then
    echo "FAIL $tag/$config produced $got verdicts for $n problems." >&2
    exit 1
  fi
}

t0=$(date +%s)
for c in "${CONFIGS[@]}"; do
  sweep "$CHOMP_BIN" chomp "$c" "$WORKDIR/$c.json" "$WORKDIR/$c.chomp.ndjson" "$SHARDS"
done
chomp_wall=$(( $(date +%s) - t0 ))
echo "chomp wall=${chomp_wall}s" >&2

# STOMP gets one process per problem, not $SHARDS. Its cost distribution is
# bimodal -- a problem that reaches a valid trajectory stops immediately
# (`num_iterations_after_valid` = 0), a problem that never does runs all
# 1000 iterations, and the second mode is three orders of magnitude more
# expensive than the first. Under $SHARDS-way splitting the wall clock is
# (problems per shard) x (the shard's worst problem); one process per
# problem makes it just the single worst problem, which is the floor no
# arrangement can beat. Measured on the full 500 on this machine: 86min at
# 50 shards for `floor_wall` against 83min at 250 shards for the harder
# `cage`.
t0=$(date +%s)
if [[ "$TIER" != "chomp" ]]; then
  for c in "${CONFIGS[@]}"; do
    if [[ "$TIER" == "full" ]]; then
      cp "$WORKDIR/$c.json" "$WORKDIR/$c.stomp.in.json"
    else
      jq --argjson k "$STOMP_TIER1_PREFIX" '.problems |= .[0:$k]' "$WORKDIR/$c.json" \
        >"$WORKDIR/$c.stomp.in.json"
    fi
    sweep "$STOMP_BIN" stomp "$c" "$WORKDIR/$c.stomp.in.json" "$WORKDIR/$c.stomp.ndjson" \
      "$(jq '.problems | length' "$WORKDIR/$c.stomp.in.json")"
  done
fi
stomp_wall=$(( $(date +%s) - t0 ))
echo "stomp wall=${stomp_wall}s" >&2

# --- verdicts -------------------------------------------------------------

cat "$WORKDIR"/*.chomp.ndjson >"$WORKDIR/chomp.all.ndjson"
# Created empty at tier `chomp`, where no STOMP sweep ran: the reporter
# below is told the tier and skips STOMP entirely, so an empty file here is
# never read as "STOMP solved nothing".
: >"$WORKDIR/stomp.all.ndjson"
if [[ "$TIER" != "chomp" ]]; then
  cat "$WORKDIR"/*.stomp.ndjson >"$WORKDIR/stomp.all.ndjson"
fi
jq -s -c '[.[0].result.problems[], .[1].result.problems[]]' \
  "$WORKDIR/${CONFIGS[0]}.response.json" "$WORKDIR/${CONFIGS[1]}.response.json" \
  >"$WORKDIR/cpp.all.json"

python3 - \
  "$WORKDIR/cpp.all.json" "$WORKDIR/chomp.all.ndjson" "$WORKDIR/stomp.all.ndjson" \
  "$TIER" \
  "$EXPECTED_CPP_SOLVED" "$EXPECTED_CPP_MEDIAN" \
  "$EXPECTED_CHOMP_SOLVED" "$EXPECTED_CHOMP_COND2" "$EXPECTED_CHOMP_MEDIAN" \
  "$EXPECTED_STOMP_SOLVED" "$EXPECTED_STOMP_COND2" "$EXPECTED_STOMP_MEDIAN" \
  "$EXPECTED_STOMP50_SOLVED" "$EXPECTED_STOMP50_COND2" "$EXPECTED_STOMP50_MEDIAN" \
  <<'PY'
import json
import statistics
import sys

(cpp_path, chomp_path, stomp_path, tier,
 e_cpp_solved, e_cpp_median,
 e_chomp_solved, e_chomp_cond2, e_chomp_median,
 e_stomp_solved, e_stomp_cond2, e_stomp_median,
 e_stomp50_solved, e_stomp50_cond2, e_stomp50_median) = sys.argv[1:16]

fails = []

def check(what, got, expected):
    # Repr, not a formatted float: these are exact re-derivations, and a
    # printed 2.16 that is really 2.1600000000000001 is the shape of
    # "agreement" this gate exists to refuse.
    ok = repr(got) == expected if isinstance(got, float) else str(got) == expected
    print(f"{'ok  ' if ok else 'FAIL'} {what}: {got!r} (pinned {expected})")
    if not ok:
        fails.append(what)

cpp = json.load(open(cpp_path))
cpp_solved = [p for p in cpp if p["exact"] is True]
check("cpp solved", len(cpp_solved), e_cpp_solved)
cpp_median = statistics.median(sorted(p["length"] for p in cpp_solved))
check("cpp median length", cpp_median, e_cpp_median)

# The two bars §5 states, derived from the C++ numbers this run just
# measured rather than from the pinned ones -- if the baseline moved, the
# bars move with it and the pinned-value check above is what reports it.
rate_bar = 0.9 * len(cpp_solved) / len(cpp)
length_bar = 1.3 * cpp_median

def report(name, rows, e_solved, e_cond2, e_median):
    solved = [r for r in rows if r["solved"]]
    check(f"{name} solved", len(solved), e_solved)
    cond2 = [r for r in solved if r["condition2_valid"]]
    check(f"{name} cond2 valid", len(cond2), e_cond2)
    median = statistics.median(sorted(r["length"] for r in solved))
    check(f"{name} median length", median, e_median)

    rate = len(solved) / len(rows)
    c1 = rate >= rate_bar
    c2 = len(solved) > 0 and len(cond2) == len(solved)
    c3 = median <= length_bar
    print(f"     {name} condition 1 {'MET' if c1 else 'UNMET'}: "
          f"{len(solved)}/{len(rows)} = {100*rate:.1f}% vs bar {100*rate_bar:.2f}%")
    print(f"     {name} condition 2 {'MET' if c2 else 'UNMET'}: "
          f"{len(cond2)}/{len(solved)} paths valid")
    print(f"     {name} condition 3 {'MET' if c3 else 'UNMET'}: "
          f"median {median:.4f} vs bar {length_bar:.4f}")

report("chomp", [json.loads(l) for l in open(chomp_path)],
       e_chomp_solved, e_chomp_cond2, e_chomp_median)

stomp_rows = [json.loads(l) for l in open(stomp_path)]
if tier == "full":
    report("stomp", stomp_rows, e_stomp_solved, e_stomp_cond2, e_stomp_median)
elif tier == "chomp":
    print("     stomp: NOTHING re-derived at this tier -- neither the "
          "50-problem prefix nor the full 500 was run "
          "(MOVEIT_RS_PHASE8_BENCHMARK=1 or =full runs them)")
else:
    report("stomp-50", stomp_rows,
           e_stomp50_solved, e_stomp50_cond2, e_stomp50_median)
    print("     stomp: full-500 numbers NOT re-derived at this tier "
          "(MOVEIT_RS_PHASE8_BENCHMARK=full re-derives them)")

if fails:
    print(f"FAIL {len(fails)} pinned value(s) no longer reproduce: "
          + ", ".join(fails), file=sys.stderr)
    sys.exit(1)
PY

echo "OK phase 8 CHOMP/STOMP property conditions re-derived (tier=$TIER, shards=$SHARDS, wall=$(( $(date +%s) - started ))s)"
