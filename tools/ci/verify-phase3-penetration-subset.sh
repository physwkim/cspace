#!/bin/bash
# PORTING-PLAN.md §5 Phase 3's `distance: f64` clause on the branch the main
# sweep prints but does not score: the penetration branch, `distance <= 0` at
# `moveit_core/collision_detection_fcl/src/collision_common.cpp:636`.
#
# `verify-phase3-collision-sweep.sh` leaves that branch unscored because all
# three of this repo's filed `distanceCallback` defects live in it, so an
# arbitrary penetrating state's oracle value may have come out of a defective
# path. That is a fact about the reference on an arbitrary state, not about the
# branch: each of the three has a firing condition readable in upstream's own
# source, and a query that satisfies none of them is comparable. This script is
# that measurement.
#
# The three exclusions and the source each is read from are in
# `tools/moveit-diff/src/bin/penetration_subset.rs`'s module doc; in one line
# each:
#
#   - threshold-suppresses-deeper-pairs needs a pair visited AFTER the running
#     minimum went non-positive -> excluded by allowing every robot/world pair
#     away but one, so there is no later pair;
#   - sentinel-survives-zero-contacts needs `fcl::collide` to find no contact
#     for a pair `fcl::distance` called non-positive -> excluded by using only
#     sphere x {sphere, box, cylinder}, whose libccd distance and intersect
#     routines test one and the same predicate;
#   - max-contact-depth needs a contact set with two members to pick the wrong
#     one from -> excluded by the same three pairs, whose intersect routines
#     each have exactly one `emplace_back`, not in a loop.
#
# The tolerance is the clause's own `1e-4` and is not widened here. What is
# restricted is the population, and the row this feeds says so.
#
# Robots: every committed fixture that HAS such a pair, which means every one
# with a link carrying exactly one primitive collision shape. panda, fanuc and
# dual_arm_panda have none -- every link of theirs is a single mesh -- so they
# are absent for a stated reason rather than unlisted, and the binary errors
# rather than passing when handed one.
#
# Needs docker (through `sg`, per this repo's wrapper rule) and the
# digest-gated oracle image. Without them it SKIPs loudly: a silent skip is
# indistinguishable from a pass.
#
#   sg docker -c tools/ci/verify-phase3-penetration-subset.sh
#   sg docker -c 'tools/ci/verify-phase3-penetration-subset.sh 300 7'
#
#   tools/ci/verify-phase3-penetration-subset.sh [STATES] [SEED]
#
# STATES is per target link, so a robot's request count is STATES x targets.
# The oracle draws the states from its own `random_states` at SEED, and this
# side folds SEED into the ChaCha8 stream its probe radii and offsets come
# from, so one (STATES, SEED) pair replays the whole corpus on both sides.
#
# MEASURED wall clock at the defaults, from this script's own per-robot lines
# over FOUR full runs (2026-08-06), plus ~14s for the release build. A range
# over n runs rather than one figure, because these are wall clocks on a shared
# machine: the four totals were 79s, 70s, 66s and 81s, and a single number from
# any one of them would read as a property of the corpus. The spread is the
# machine's; the request counts below are the corpus's and do not move.
#
#     prbt      2-3s  (400 req)   one_robot  2-4s  (1800 req)
#     prbt_pg70 5-10s (2100 req)  pr2       56-67s (1700 req)
#     ------------------------------------------------------
#     total     66-81s
#
# pr2 is 81% of it and is not the largest corpus -- the cost tracks the
# oracle's per-request `PlanningScene` diff over a 95-link model, not the
# number of samples kept. Do not infer a per-robot cost from sample count.
# This is the same class of cost as `verify-oracle-sweep.sh` (113s), which
# `verify-all.sh` runs unconditionally, so this one is not opt-in either.
set -uo pipefail

STATES="${1:-}"
SEED="${2:-1}"

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"

. "$REPO_ROOT/tools/ci/gate-lib.sh"
require_caller_tree "$REPO_ROOT"
# This script runs without -e on purpose, so a failed cd would not abort it
# and every path below would resolve against the caller's directory instead.
cd "$REPO_ROOT" || exit 1

BIN="$REPO_ROOT/target/release/penetration_subset"

# Per-robot, because the corpora are very different sizes at equal STATES: pr2
# has 17 target links to prbt's 2, and equalising the request counts is what
# keeps pr2 from being ten minutes on its own. A caller-supplied STATES
# overrides all of them.
declare -A DEFAULT_STATES=(
  [prbt]=200
  [prbt_pg70]=300
  [one_robot]=300
  [pr2]=100
)
ROBOTS=(prbt prbt_pg70 one_robot pr2)

if ! command -v docker >/dev/null 2>&1; then
  echo "SKIP docker is not on PATH -- §5 Phase 3's penetration branch is not measured by this run."
  echo "SKIP this is not a pass."
  exit 0
fi

# shellcheck source=tools/moveit-oracle/src-digest.sh
source "$REPO_ROOT/tools/moveit-oracle/src-digest.sh"
want="$(oracle_stamp "$REPO_ROOT/tools/moveit-oracle")"
IMAGE="${IMAGE:-$(oracle_image_tag "$want")}"
stamp="$(oracle_stamp_verdict "$IMAGE" "$want")"
if [ "$stamp" != ok ]; then
  # A docker this shell cannot reach is not a skip -- nothing was measured.
  # `oracle_stamp_explain` returns nonzero for exactly that cause, because
  # `verify-all.sh` reads each gate's exit status and not these lines, so
  # exiting 0 would report it as a pass.
  oracle_stamp_explain "$stamp" "$IMAGE" "$want" "SKIP " || exit 1
  echo "SKIP this is not a pass -- the oracle was never consulted."
  exit 0
fi

# Release, not debug: the Rust side runs one `distance_robot` per request and
# an unoptimised build makes it, rather than the oracle, the bottleneck.
if ! cargo build --release --manifest-path "$REPO_ROOT/Cargo.toml" -p moveit-diff \
  --bin penetration_subset; then
  echo "FAIL could not build penetration_subset" >&2
  exit 1
fi

OUT_DIR="$(mktemp -d)"
trap 'rm -rf "$OUT_DIR"' EXIT

echo
echo "=== §5 Phase 3 distance clause, penetration branch, defect-free subset ==="
echo "    tolerance 1e-4 -- the clause's own, not widened"
echo "    corpus: one masked robot/world pair, sphere x {sphere, box, cylinder}"
echo

status=0
declare -a SUMMARY=()

for robot in "${ROBOTS[@]}"; do
  states="${STATES:-${DEFAULT_STATES[$robot]}}"
  echo "--- $robot ($states states/target, seed $SEED) ---"
  out="$OUT_DIR/$robot.out"

  # Redirected to a file, never piped: a pipeline reports the filter's status,
  # which is how a disagreement becomes a silent pass.
  start="$SECONDS"
  "$BIN" \
    --urdf "$REPO_ROOT/fixtures/$robot.urdf" \
    --srdf "$REPO_ROOT/fixtures/$robot.srdf" \
    --oracle "$REPO_ROOT/tools/moveit-oracle/run-oracle.sh" \
    --states "$states" \
    --seed "$SEED" \
    --tol 1e-4 \
    > "$out" 2>&1
  rc=$?
  elapsed=$((SECONDS - start))

  grep -vE '^\[(WARN|INFO|ERROR)\]' "$out"
  echo "wall clock: ${elapsed}s"
  echo

  # The verdict line, not the exit code, decides -- a run killed mid-corpus
  # exits nonzero having compared nothing, and calling that "disagreed" sends
  # the reader after a numeric bug that is not there. See `run_verdict`.
  verdict="$(run_verdict "$rc" "$out" '^(NOT )?MET at tol')"
  case "$verdict" in
    ok) SUMMARY+=("$robot: MET (${elapsed}s)") ;;
    disagreed)
      SUMMARY+=("$robot: NOT MET (${elapsed}s)")
      status=1
      ;;
    *)
      SUMMARY+=("$robot: $verdict (${elapsed}s)")
      status=1
      ;;
  esac
done

require_nonempty "${#SUMMARY[@]}" "robots to measure"

echo "=== summary ==="
printf '  %s\n' "${SUMMARY[@]}"

if [[ "$status" -ne 0 ]]; then
  echo "FAIL §5 Phase 3's distance clause is not met on the defect-free penetration subset." >&2
  exit 1
fi

echo "OK §5 Phase 3's distance clause holds at 1e-4 on the defect-free penetration subset."
