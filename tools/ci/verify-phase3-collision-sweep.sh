#!/bin/bash
# PORTING-PLAN.md §5 Phase 3's completion condition, as a command rather than
# a number in a report: 10,000 random states per robot, with
#
#   - `collision: bool` compared for exact equality (the condition's "100%
#     일치" clause), and
#   - `distance: f64` compared within `1e-4` (the condition's second clause),
#
# against the C++ oracle. The two clauses are counted and reported
# separately -- see `CollisionClauseStats` in `tools/moveit-diff/src/main.rs`
# for why one combined `failed:` total cannot express this condition.
#
# The distance clause is judged on ONE side of a branch in the oracle's own
# code, and this is the script that has to say so out loud. `distanceCallback`
# publishes two different quantities through the single `distance` field:
# above `moveit_core/collision_detection_fcl/src/collision_common.cpp:636`
# (`if (distance <= 0 && cdata->req->enable_signed_distance)`) it is
# `fcl::distance`'s own return, a separation distance; below it the value is
# discarded and re-derived by picking one contact out of an `fcl::collide`
# set (`:663`). All three of this port's filed divergences from that function
# -- `doc/upstream-bugs.md`'s `distance-callback-max-contact-depth`,
# `distance-callback-threshold-suppresses-deeper-pairs` and
# `fcl-distance-sentinel-survives-zero-contacts` -- are defects of the second
# and cannot fire in the first. So the verdict below rests on the separated
# branch; the penetration branch is measured, printed on every run, and not
# scored HERE. It is scored elsewhere, and that is the one thing this header
# must not be read as saying it is not: `verify-phase3-penetration-subset.sh`
# takes each of the three defects' firing conditions from upstream's own
# source and scores the sub-population where none of them can fire -- one
# masked pair per query, sphere against sphere/box/cylinder -- at this same
# `1e-4`. What stays unscored anywhere is the rest of the branch: a query with
# two or more pairs, `box x box`, and meshes. That is argued and measured in
# the round section cited by
# PORTING-PLAN.md §5's `distance: f64` row, which also carries the two
# mutations showing what each branch is still guarded by. Cited by row rather
# than by number on purpose: a worker cannot know the number its section will
# be assigned at merge, and the unassigned-placeholder scan in
# `check-porting-plan-sections.sh` now reads every tracked file as bytes, so
# writing one into this script would fail the gate rather than sit here
# unfilled -- which is what citing by row avoids needing.
#
# What is restricted is the compared *population*, never the tolerance. `1e-4`
# is the condition's own number and stays; every separated side that misses it
# is counted and turns the run red. A future round that wants to widen `1e-4`
# instead should read §11.10 first.
#
# Contact-point coordinates are NOT compared. That exclusion is the
# condition's own third bullet -- "접촉점 좌표는 비교 대상에서 제외 (§4.5,
# 검증 한계로 기록)" -- recorded in §4.5 as a verification limit of this
# port, not a convenience taken here to make a number pass. The two sides'
# contact geometry differs by construction (`crates/moveit-collision/
# src/parry.rs`, deviations 4 and 6); §4.5 is where that is argued, and this
# script is not the place it is decided.
#
# Sibling of `verify-oracle-sweep.sh`, which is the same shape for Phase 2
# (FK at `1e-9`, jacobian at `1e-7`) and which this script deliberately does
# not extend: that one is a per-round regression gate measured at 113s over
# the same 10000 states, and folding collision into it would take it to
# ~4928s -- 43x -- while hiding two distinct completion conditions behind
# one exit code.
#
# OPT-IN, and why it is not simply in `verify-all.sh`'s glob like the rest:
# the full condition run costs 4815s (80m15s) of wall clock on this machine
# -- see the measured per-robot table at the bottom of this header.
# `verify-all.sh` runs every `tools/ci/verify-*.sh` by glob on every merge
# round; an unconditional 80-minute member would dominate the round's cost.
# So it SKIPs unless `PHASE3_SWEEP=1`, in the loud shape
# `verify-mpr-vs-epa.sh` already uses for its own expensive precondition: a
# silent skip is indistinguishable from a pass, which is the failure
# `verify-vendored-fixture-tests.sh` documents at length.
#
# (There is no env-var opt-in convention in this directory to copy: every
# other environment variable read by a `tools/ci` script -- `MOVEIT2_SRC`,
# `LIBCCD_SRC`, `OCTOMAP_SRC` -- names the path of an external checkout, not
# an opt-in. `PHASE3_SWEEP` is a cost gate on the same loud-SKIP mechanism,
# and sweep *size* stays where `verify-oracle-sweep.sh` already puts it,
# in positional arguments.)
#
#   PHASE3_SWEEP=1 sg docker -c tools/ci/verify-phase3-collision-sweep.sh
#   PHASE3_SWEEP=1 sg docker -c 'tools/ci/verify-phase3-collision-sweep.sh 200 7'
#
#   tools/ci/verify-phase3-collision-sweep.sh [CASES] [SEED]
#
# The state sampling is seeded and reproducible: every state comes from the
# oracle's own `random_states` op at the seed below, so the same (CASES,
# SEED) pair replays the identical 10,000 states on both sides. The seed is
# printed in this script's own output, per robot, so a reported number
# carries the seed that produced it.
#
# Exits non-zero if either clause is unmet on any robot -- like
# `verify-oracle-sweep.sh`, a completion-condition check reports the
# condition, and reports every robot before it does.
#
# MEASURED wall clock, from this script's own per-robot "wall clock" lines on
# its first full run (2026-08-05, `PHASE3_SWEEP=1 ... 10000 1`):
#
#     panda            438s     prbt              18s
#     fanuc           2829s     dual_arm_panda   454s
#     pr2             1076s
#     ------------------------------------------------
#     total           4815s  (80m15s), plus build + provenance check
#
# `fanuc` is 59% of the total on its own and is why this is opt-in: it is not
# the largest robot (9 links to pr2's 95), so the cost tracks pairs actually
# reaching narrowphase, not model size. Do not infer a per-robot cost from
# link count when scheduling this.
#
# Measured on a 96-core host at loadavg ~10, with a sibling caucus panel
# running its own sweep concurrently. `moveit-diff` is single-threaded and
# held ~98% of one core throughout, and load stayed far below core count, so
# these are not contention-inflated -- but they are single-core times and do
# not improve if you give the machine more cores.
set -uo pipefail

CASES="${1:-10000}"
SEED="${2:-1}"

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"

. "$REPO_ROOT/tools/ci/gate-lib.sh"
require_caller_tree "$REPO_ROOT"
DIFF="$REPO_ROOT/target/release/moveit-diff"

if [[ "${PHASE3_SWEEP:-}" != "1" ]]; then
  echo "SKIP PHASE3_SWEEP is not 1 -- the ${CASES}-state collision sweep did not run."
  echo "SKIP this is not a pass; §5 Phase 3's completion condition is unmeasured by this run."
  echo "SKIP run it with: PHASE3_SWEEP=1 sg docker -c $0"
  exit 0
fi

# Every committed robot description, not only the condition's named three
# (panda / prbt / fanuc): dual_arm_panda and pr2 are committed fixtures whose
# collision geometry differs in kind from those three (pr2 is the only
# fixture with primitive *and* mesh collision shapes; dual_arm_panda is the
# only one with two arms), and a condition met on a subset of the committed
# fixtures is not a fact about this port's collision layer. The per-robot
# report below is what the three-robot condition is read off.
ROBOTS=(panda prbt fanuc dual_arm_panda pr2)

# Before comparing anything: confirm the fixtures still are the robots they
# name -- same reasoning as `verify-oracle-sweep.sh`'s own call. A sweep that
# agrees with the oracle on a drifted `fixtures/panda.urdf` proves both sides
# read the same file, not that either matches upstream panda.
if ! "$REPO_ROOT/tools/ci/verify-fixture-provenance.sh"; then
  echo "FAIL fixture provenance check failed -- no sweep result below would mean anything" >&2
  exit 1
fi

# Release, not debug: 10k states x 5 robots is ~50k collision checks per
# side, and the debug build makes the Rust side rather than the oracle the
# bottleneck.
if ! cargo build --release --manifest-path "$REPO_ROOT/Cargo.toml" -p moveit-diff; then
  echo "FAIL could not build moveit-diff" >&2
  exit 1
fi

OUT_DIR="$(mktemp -d)"
trap 'rm -rf "$OUT_DIR"' EXIT

echo
echo "=== §5 Phase 3 completion condition: $CASES states x ${#ROBOTS[@]} robots, seed $SEED ==="
echo "    collision: bool -- exact equality"
echo "    distance:  f64  -- within 1e-4, on the sides where the oracle publishes"
echo "                       fcl::distance's own return (collision_common.cpp:636)"
echo "    distance:  f64  -- on the penetration side: measured and printed, not scored"
echo "                       here; scored by verify-phase3-penetration-subset.sh on the"
echo "                       sub-population where none of the three defects can fire"
echo "    contact-point coordinates -- excluded per §4.5 (recorded verification limit)"
echo

status=0
declare -a SUMMARY=()

for robot in "${ROBOTS[@]}"; do
  echo "--- $robot ($CASES cases, seed $SEED) ---"
  out="$OUT_DIR/$robot.out"
  stats="$OUT_DIR/$robot.json"

  # Redirected to a file rather than piped into a filter: a pipe reports the
  # filter's status, which turns a disagreement into a silent pass. The
  # status is captured rather than left to `set -e` (which this script does
  # not set, for the same reason) so every robot is measured and reported
  # even after an earlier one disagreed -- a condition check that stops at
  # the first failure cannot say how far from met the condition is.
  start="$SECONDS"
  "$DIFF" \
    --urdf "$REPO_ROOT/fixtures/$robot.urdf" \
    --srdf "$REPO_ROOT/fixtures/$robot.srdf" \
    --cases "$CASES" \
    --seed "$SEED" \
    --collision \
    --tol-distance 1e-4 \
    --stats-json "$stats" \
    --oracle "$REPO_ROOT/tools/moveit-oracle/run-oracle.sh" \
    > "$out" 2>&1
  rc=$?
  elapsed=$((SECONDS - start))

  if [[ ! -s "$stats" ]]; then
    echo "FAIL $robot produced no --stats-json (exit $rc); last 20 lines:" >&2
    tail -20 "$out" >&2
    status=1
    SUMMARY+=("$robot: NO RESULT (exit $rc)")
    continue
  fi

  # Read straight out of --stats-json rather than re-grepping stdout: the
  # numbers reported here are then the same objects moveit-diff counted,
  # not a second parse of their printed form.
  #
  # Captured with `if ! line="$(...)"` rather than `read < <(...)`: a python
  # that failed would otherwise leave every field empty, and empty is not
  # "0", so the row below would read UNMET -- reporting a broken run as a
  # measured failure of the condition. That is the `set -e`-cannot-see-it
  # shape `gate-lib.sh` documents; here the distinction matters because one
  # of the two outcomes is a claim about the port.
  if ! line="$(python3 - "$stats" "$CASES" "$elapsed" <<'PY'
import json, sys
s = json.load(open(sys.argv[1]))
cases, elapsed = sys.argv[2], sys.argv[3]
c = s["collision_clauses"]
sep, pen = c["separated"], c["penetrating"]
print("%s|%s|%s|%s|%s|%s|%.6e|%s|%s|%s|%.6e|%s|%s|%.6e" % (
    c["bool_disagrees"], c["bool_total"],
    c["distance_disagrees"], c["distance_total"],
    c["errored"], cases,
    s["worst_distance_deviation"], elapsed,
    sep["disagrees"], sep["total"], sep["worst_deviation"],
    pen["disagrees"], pen["total"], pen["worst_deviation"]))
PY
  )"; then
    echo "FAIL $robot: --stats-json exists but could not be read for the two clauses" >&2
    status=1
    SUMMARY+=("$robot: NO RESULT (stats unreadable)")
    continue
  fi
  IFS='|' read -r bool_bad bool_tot _dist_bad dist_tot errored _cases worst secs \
    sep_bad sep_tot sep_worst pen_bad pen_tot pen_worst <<<"$line"

  sed -n '/^worst distance deviation/,/^robot same-pair/p' "$out"
  echo "wall clock: ${secs}s"
  echo

  verdict="met"
  # The distance clause is judged on the separated branch, which is where the
  # oracle publishes `fcl::distance`'s own return. `$pen_bad` is measured and
  # printed on every run but is not part of the verdict -- see this script's
  # header for why, and note that this is a restriction of the compared
  # population, never of the tolerance: `1e-4` is unchanged and `$sep_bad`
  # counts every side that misses it.
  if [[ "$bool_bad" != "0" || "$sep_bad" != "0" || "$errored" != "0" ]]; then
    verdict="UNMET"
    status=1
  fi
  SUMMARY+=("$(printf '%-15s bool %6s/%-6s  dist(sep) %6s/%-6s worst %-12s  dist(pen, unjudged) %6s/%-6s worst %-12s  %ss  %s' \
    "$robot" "$bool_bad" "$bool_tot" "$sep_bad" "$sep_tot" "$sep_worst" \
    "$pen_bad" "$pen_tot" "$pen_worst" "$secs" "$verdict")")
done

echo "=== §5 Phase 3 summary (seed $SEED, $CASES states/robot; disagreements/total) ==="
printf '%s\n' "${SUMMARY[@]}"
echo

if [[ $status -ne 0 ]]; then
  echo "§5 Phase 3's completion condition is NOT met -- see the per-robot rows above." >&2
  echo "The tolerance is the condition's own 1e-4 and is not to be widened to close this:" >&2
  echo "a divergence larger than 1e-4 is a finding about the collision backend." >&2
  echo "Neither is the judged population to be narrowed further: the separated branch is" >&2
  echo "where the oracle reports the quantity the clause names, and a miss there is the" >&2
  echo "port's." >&2
  exit 1
fi

echo "§5 Phase 3's completion condition is met on all ${#ROBOTS[@]} robots."
