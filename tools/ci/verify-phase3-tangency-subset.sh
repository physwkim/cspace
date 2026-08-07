#!/bin/bash
# PORTING-PLAN.md §5 Phase 3's `collision: bool` clause on the sub-population
# where neither narrow phase's boundary convention can decide the answer.
#
# `verify-phase3-collision-sweep.sh` leaves that clause UNMET for one reason:
# `fcl::collide` dispatches per shape pair, and the pairs it leaves to libccd
# MPR answer `false` at exact contact while the pairs it specialises answer
# `true`. prbt's `cylinder x box` is one of the unspecialised ones, so all
# 6,854 of that sweep's disagreements are a single blank cell sampled 10,000
# times. That is a fact about which pairs the reference is uniform on, not
# about the clause. This script measures the pairs where both sides run the
# same closed form on the same side of the boundary.
#
# The derivation is in `tools/moveit-diff/src/bin/tangency_subset.rs`'s module
# doc, with the citations; in one line each:
#
#   - upstream is uniform only on the pairs registered at
#     gjk_solver_libccd-inl.h:245-267, everything else being libccd MPR, which
#     tests interior overlap strictly;
#   - this port is uniform only on the pairs parry's DefaultQueryDispatcher
#     sends to a closed form, everything else being a GJK whose boundary sits
#     in a positive gap that §229.2 measured at ~5e-8 m;
#   - the two sets overlap on sphere x {sphere, box, cylinder}, and
#     sphere x sphere drops out because the two closed forms take opposite
#     sides of the boundary (fcl's `len > r1 + r2` reject includes contact,
#     parry's `distance_squared < ...` accept excludes it).
#
# So: `sphere x {box, cylinder}`, either order. `box x box`, `box x cylinder`,
# `cylinder x cylinder` and `sphere x sphere` are measured as CONTROL arms and
# never scored -- they are what shows the corpus can separate the two dispatch
# tables at the same gaps where the scored pairs agree.
#
# There is no tolerance to widen here: the quantity is a boolean. What is
# restricted is the population, and the row this feeds says so.
#
# Robots: every committed fixture that HAS such a pair, which means every one
# with a link carrying exactly one primitive collision shape. panda, fanuc and
# dual_arm_panda have none -- every link of theirs is a single mesh -- so they
# are absent for a stated reason rather than unlisted, and the binary errors
# rather than passing when handed one.
#
# one_robot is absent for the same shape of reason, not the same reason: it
# HAS scored pairs (sphere x {box, cylinder} against its own box-shaped
# links), but its six links are all the identical box half [0.5, 1.0, 0.5]
# (fixtures/one_robot.urdf), so every control-arm sample this binary can draw
# for it -- box x box and box x cylinder -- lands nowhere near the libccd/GJK
# boundary the control arms exist to exercise. Measured at both the former
# default (170 states/target, 858 control samples) and at 1200
# (~7x, 5996 control samples): 0 disagree either way, where every other
# robot here disagrees on 3-8% of its control samples at the default. That is
# a property of this fixture's uniform box geometry, not a thin corpus --
# more states does not buy the pair back its power, and `tangency_subset`'s
# own `libccd_disagrees == 0` check refuses to certify a boolean the corpus
# cannot separate (`tools/moveit-diff/src/bin/tangency_subset.rs`). Dropping
# it here, rather than reporting it as a distinct non-fatal verdict per
# robot, keeps that refusal a property of the fixture roster instead of a
# string this script has to recognise in a comparison binary's stderr to
# know it is not a real failure -- `oracle_stamp_verdict`'s own doc names
# exactly that trap (`grep -n 'reworded error message' tools/ci/gate-lib.sh`).
# If a future edit to fixtures/one_robot.urdf gives it varied link shapes,
# re-add it and re-measure.
#
# Needs docker (through `sg`, per this repo's wrapper rule) and the
# digest-gated oracle image. Without them it SKIPs loudly: a silent skip is
# indistinguishable from a pass.
#
#   sg docker -c tools/ci/verify-phase3-tangency-subset.sh
#   sg docker -c 'tools/ci/verify-phase3-tangency-subset.sh 40 7'
#
#   tools/ci/verify-phase3-tangency-subset.sh [STATES] [SEED]
#
# STATES is per target link, and each state draws one placement per probe
# shape, so a robot's request count is STATES x targets x 3 plus one forward
# kinematics request per state. The oracle draws the states from its own
# `random_states` at SEED, and this side folds SEED into the ChaCha8 stream
# its placements come from, so one (STATES, SEED) pair replays the whole
# corpus on both sides.
#
# MEASURED wall clock at the defaults, from this script's own per-robot lines
# over THREE full runs (2026-08-06), plus ~15s for the release build. Ranges
# rather than single figures, because these are wall clocks on a shared
# machine; the three totals happened to come out 90s, 90s and 90s, which is
# the spread being small today and not a promise. The request counts are the
# corpus's and do not move. one_robot's 5-6s (3060 req) row is subtracted
# below, arithmetically, since dropping it from ROBOTS (see "Robots:" above)
# was not re-measured as a fourth full run.
#
#     prbt       4-5s  (1200 req)
#     prbt_pg70 11-14s (2688 req)
#     pr2       67-68s (1632 req)
#     ----------------------------
#     total     ~84s
#
# pr2 is ~80% of it (arithmetic, from the row above) on ~30% of the
# remaining requests (1632 of 1200+2688+1632=5520) -- the cost tracks the oracle's
# per-request `PlanningScene` diff over a 95-link model, not the number of
# samples kept. Do not infer a per-robot cost from sample count. This is the
# same class of cost as `verify-oracle-sweep.sh` (113s), which `verify-all.sh`
# runs unconditionally, so this one is not opt-in either.
set -uo pipefail

STATES="${1:-}"
SEED="${2:-1}"

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"

. "$REPO_ROOT/tools/ci/gate-lib.sh"
require_caller_tree "$REPO_ROOT"
# This script runs without -e on purpose, so a failed cd would not abort it
# and every path below would resolve against the caller's directory instead.
cd "$REPO_ROOT" || exit 1

BIN="$REPO_ROOT/target/release/tangency_subset"

# Per-robot, because the corpora are very different sizes at equal STATES: pr2
# has 17 target links to prbt's 2, and equalising the scored sample counts is
# what keeps pr2 from dominating the wall clock even further than it does. A
# caller-supplied STATES overrides all of them.
declare -A DEFAULT_STATES=(
  [prbt]=200
  [prbt_pg70]=128
  [pr2]=32
)
ROBOTS=(prbt prbt_pg70 pr2)

if ! command -v docker >/dev/null 2>&1; then
  skip_not_measured blocked \
    "docker is not on PATH -- §5 Phase 3's collision clause is not measured by this run." \
    "this is not a pass."
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
  skip_not_measured blocked "this is not a pass -- the oracle was never consulted."
fi

# Release, not debug: the Rust side builds a fresh `ParryCollisionEnv` per
# request and an unoptimised build makes it, rather than the oracle, the
# bottleneck.
if ! cargo build --release --manifest-path "$REPO_ROOT/Cargo.toml" -p moveit-diff \
  --bin tangency_subset; then
  echo "FAIL could not build tangency_subset" >&2
  exit 1
fi

OUT_DIR="$(mktemp -d)"
trap 'rm -rf "$OUT_DIR"' EXIT

echo
echo "=== §5 Phase 3 collision clause, comparable-dispatch subset ==="
echo "    a boolean has no tolerance -- what is restricted is the population"
echo "    scored: one masked robot/world pair, sphere x {box, cylinder}"
echo "    control, never scored: box x box, box x cylinder, cylinder x cylinder,"
echo "                           sphere x sphere, and every gap of exactly zero"
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
    > "$out" 2>&1
  rc=$?
  elapsed=$((SECONDS - start))

  grep -vE '^\[(WARN|INFO|ERROR)\]' "$out"
  echo "wall clock: ${elapsed}s"
  echo

  # The verdict line, not the exit code, decides -- a run killed mid-corpus
  # exits nonzero having compared nothing, and calling that "disagreed" sends
  # the reader after a semantic bug that is not there. See `run_verdict`.
  verdict="$(run_verdict "$rc" "$out" '^(MET on|NOT MET)')"
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
  echo "FAIL §5 Phase 3's collision clause is not met on the comparable-dispatch subset." >&2
  exit 1
fi

echo "OK §5 Phase 3's collision clause holds on the comparable-dispatch subset."
