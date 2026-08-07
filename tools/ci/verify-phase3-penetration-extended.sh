#!/bin/bash
# Phase 3's `distance: f64` clause on the two penetration subpopulations
# `verify-phase3-penetration-subset.sh` leaves out by construction: two or
# more simultaneously penetrating pairs, and `box x box`. (The condition
# itself now lives in `GOALS.md`, not `PORTING-PLAN.md` -- that file and
# `doc/upstream-bugs.md` were both deleted at `f7186386` in favour of a
# single goals file; GOALS.md's row still carries the same carve-out:
# "관통 분기는 상류 결함이 발화할 수 없는 부분모집단에서 1e-4 이내".)
#
# `tools/moveit-diff/src/bin/penetration_extended.rs` and
# `tools/moveit-oracle/src/oracle.cpp`'s `pair_signed_distance` op were built
# at `a15fc5c3` to measure exactly these two subpopulations. Until this
# script, neither had a CI gate or a recorded verdict anywhere in this
# repository -- `git log -S 'box x box: MET'` and
# `git log -S 'multi-pair: MET'` both return nothing. This script is that
# gate, and it has never itself been run (see "State of this gate" below).
#
# # Why the sibling excludes these two
#
# `verify-phase3-penetration-subset.sh`'s corpus is one masked (probe,
# target) pair at a time, sphere against a link with exactly one primitive
# collision shape -- the construction that keeps `distance-callback-
# threshold-suppresses-deeper-pairs` (needs a second pair) and
# `distance-callback-max-contact-depth` (needs a multi-contact set) from
# firing at all. That excludes, by the same construction:
#
#   - any scene with two or more pairs simultaneously penetrating, which is
#     the only configuration where the suppressed-pair defect is even
#     observable;
#   - `box x box`, because `boxBoxIntersect` can emit more than one contact
#     (`box_box-inl.h:857-874`, `boxBox2(..., 4, contacts)` at :865-868 --
#     read directly against fcl tag `df2702ca5e703dec98ebd725782ce13862e87fc8`
#     for this gate), reopening the max-contact-depth defect the same way a
#     multi-triangle mesh does.
#
# # What this script measures instead
#
# Two independent subpopulations, per `penetration_extended.rs`'s own module
# doc (read there for the full citation trail; this is the one-line version):
#
#   - **Multi-pair**: `penetration_subset.rs`'s own single-pair mask, reused
#     twice against one scene (once per target, each still seeing exactly one
#     pair, so neither call's own defect exposure is reopened) -- the true
#     per-pair distance is masked-query ground truth, and the minimum of the
#     two is the scene's true multi-pair minimum. What is new is the *port*
#     side of the comparison: `multi_pair_port_distance` asks `distance_robot`
#     once with *both* pairs unmasked, exercising the port's own minimum-of-
#     pairs logic with more than one pair live for the first time in any gate
#     in this directory.
#   - **`box x box`**: a new oracle op, `pair_signed_distance`, that bypasses
#     `distanceCallback` entirely and calls `fcl::distance` directly with
#     `enable_signed_distance = true` against MoveIt's own unmodified FCL
#     object factories. Verified directly against the pinned fcl tag for this
#     gate: `collision_request.h:102`'s `GST_LIBCCD` default and
#     `distance_request.h:68` ("primitive shapes | SD_1, NP" under
#     `GST_LIBCCD`) together mean `box x box` distance already reaches FCL's
#     native exact-signed-distance path both with and without this op --
#     `distanceCallback` itself never takes that path only because it builds
#     its FCL request with `enable_nearest_points` alone
#     (`collision_common.cpp:603`), leaving FCL's own `enable_signed_distance`
#     at its default `false`. `pair_signed_distance` is not a second
#     implementation of anything upstream computes; it is upstream's own
#     distance routine, called the way `distanceCallback` does not.
#
# # What stays OUT: mesh
#
# Neither measurement here, nor any conceivable one, extends to mesh.
# `distance_request.h:69` -- read directly against the pinned fcl tag --
# classifies "mesh and octree" as `SD_2, NP_X` under *both* solver types:
# positive distance only, negative distance left implementation-defined.
# There is no upstream call, `pair_signed_distance` or any other, that
# returns a genuine signed distance for a penetrating mesh pair to compare
# the port against -- reproducing one from this side of the wire would be
# deriving it independently, not comparing against the reference. A green
# run of this script says nothing about panda, fanuc or dual_arm_panda's own
# penetrating-branch population, which per `penetration_subset.rs`'s own
# targeting ("every link of theirs is a single mesh") is effectively all of
# it. Two hand-built reproducers exist elsewhere for specific mesh instances
# of these same defects (`penetration_depth_scale_invariance.rs`'s
# `panda_link0`-vs-floor case, `minimum_distance_is_the_minimum.rs`'s fanuc
# state 9651) and both corroborate the mechanism at real magnitude; neither
# is a population-coverage claim, and this script does not turn them into one.
#
# # Robots
#
# Identical roster and identical reason to the sibling: every fixture with a
# link carrying exactly one primitive collision shape --
# `prbt prbt_pg70 one_robot pr2`. panda, fanuc and dual_arm_panda are absent
# for the reason stated above, not unlisted.
#
# # Two empty-population guards, not one
#
# The sibling's `require_nonempty` covers "did every robot in ROBOTS get
# attempted" -- sufficient there because that binary hard-errors (exit 2) on
# a robot offering zero targets. This binary does not: a robot short of
# targets for one measurement prints `<label>: SKIPPED (nothing to measure on
# this robot)` and exits 0, which is *correct* per-robot behaviour (a robot
# with one qualifying link genuinely cannot supply a multi-pair sample) but
# means a per-robot attempted-count alone cannot catch a link-selection
# regression that empties one measurement's population across the *entire*
# roster -- every robot would print SKIPPED, every robot would exit 0, and
# the gate would read green having scored nothing. So this script additionally
# sums each measurement's own `kept` count across all four robots and calls
# `require_nonempty` on each total independently before deciding a verdict.
#
# # State of this gate: UNEXECUTED
#
# Written and reviewed, never run. The sibling scripts each carry a `MEASURED
# wall clock` section from their own first full run; this one does not,
# deliberately, rather than guess a number a run has not produced. `--states`
# is left at `penetration_extended`'s own built-in default (40) unless a
# caller overrides it, for the same reason: the sibling's per-robot
# `DEFAULT_STATES` table was hand-tuned against a measured per-robot cost this
# binary has never had taken. The first real run is expected to reveal
# whether that default is too cheap (an undersized sample masking a real
# defect) or too expensive (this binary costs two oracle round trips per
# multi-pair sample, not one) -- either finding is follow-up work, not
# something to guess at here.
#
# Needs docker (through `sg`, per this repo's wrapper rule) and the
# digest-gated oracle image. Without them it SKIPs loudly: a silent skip is
# indistinguishable from a pass.
#
#   sg docker -c tools/ci/verify-phase3-penetration-extended.sh
#   sg docker -c 'tools/ci/verify-phase3-penetration-extended.sh 100 7'
#
#   tools/ci/verify-phase3-penetration-extended.sh [STATES] [SEED]
set -uo pipefail

STATES="${1:-}"
SEED="${2:-1}"

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"

. "$REPO_ROOT/tools/ci/gate-lib.sh"
require_caller_tree "$REPO_ROOT"
# This script runs without -e on purpose, so a failed cd would not abort it
# and every path below would resolve against the caller's directory instead.
cd "$REPO_ROOT" || exit 1

BIN="$REPO_ROOT/target/release/penetration_extended"

ROBOTS=(prbt prbt_pg70 one_robot pr2)

if ! command -v docker >/dev/null 2>&1; then
  skip_not_measured blocked \
    "docker is not on PATH -- the multi-pair / box x box penetration subpopulations are not measured by this run." \
    "this is not a pass."
fi

# shellcheck source=tools/moveit-oracle/src-digest.sh
source "$REPO_ROOT/tools/moveit-oracle/src-digest.sh"
want="$(oracle_stamp "$REPO_ROOT/tools/moveit-oracle")"
IMAGE="${IMAGE:-$(oracle_image_tag "$want")}"
stamp="$(oracle_stamp_verdict "$IMAGE" "$want")"
if [ "$stamp" != ok ]; then
  # A docker this shell cannot reach is not a skip -- nothing was measured.
  oracle_stamp_explain "$stamp" "$IMAGE" "$want" "SKIP " || exit 1
  skip_not_measured blocked "this is not a pass -- the oracle was never consulted."
fi

# Release, not debug: each sample here costs two oracle round trips (the
# multi-pair measurement queries each pair in isolation before the port
# call), so an unoptimised build makes this side, not the oracle, the
# bottleneck -- the same reasoning as the sibling's own release build.
if ! cargo build --release --manifest-path "$REPO_ROOT/Cargo.toml" -p moveit-diff \
  --bin penetration_extended; then
  echo "FAIL could not build penetration_extended" >&2
  exit 1
fi

OUT_DIR="$(mktemp -d)"
trap 'rm -rf "$OUT_DIR"' EXIT

echo
echo "=== Phase 3 distance clause, penetration branch, multi-pair + box x box ==="
echo "    tolerance 1e-4 -- the clause's own, not widened"
echo "    scored: >=2 simultaneously penetrating pairs (any qualifying primitive"
echo "    pair), and box x box, on prbt/prbt_pg70/one_robot/pr2"
echo "    NOT scored: mesh, anywhere -- FCL 0.7.0 has no genuine signed distance"
echo "    for a penetrating mesh pair (distance_request.h:69, SD_2). A pass below"
echo "    says nothing about panda, fanuc or dual_arm_panda's own penetrating"
echo "    population."
echo

status=0
declare -a SUMMARY=()
multi_pair_kept_total=0
box_box_kept_total=0

for robot in "${ROBOTS[@]}"; do
  echo "--- $robot (seed $SEED) ---"
  out="$OUT_DIR/$robot.out"

  args=(--urdf "$REPO_ROOT/fixtures/$robot.urdf" --srdf "$REPO_ROOT/fixtures/$robot.srdf" \
    --oracle "$REPO_ROOT/tools/moveit-oracle/run-oracle.sh" --seed "$SEED" --tol 1e-4)
  [ -n "$STATES" ] && args+=(--states "$STATES")

  # Redirected to a file, never piped: a pipeline reports the filter's
  # status, which is how a disagreement becomes a silent pass.
  start="$SECONDS"
  "$BIN" "${args[@]}" > "$out" 2>&1
  rc=$?
  elapsed=$((SECONDS - start))

  grep -vE '^\[(WARN|INFO|ERROR)\]' "$out"
  echo "wall clock: ${elapsed}s"
  echo

  # The verdict line, not the exit code, decides -- see `run_verdict`. Each
  # measurement's own MET/NOT MET line carries its label, unlike the
  # sibling's single unlabelled line.
  verdict="$(run_verdict "$rc" "$out" '^(multi-pair|box x box): (MET|NOT MET) at tol')"
  case "$verdict" in
    ok) SUMMARY+=("$robot: ran (${elapsed}s)") ;;
    disagreed)
      SUMMARY+=("$robot: NOT MET (${elapsed}s)")
      status=1
      ;;
    *)
      SUMMARY+=("$robot: $verdict (${elapsed}s)")
      status=1
      ;;
  esac

  # Per-measurement kept counts, summed across every robot -- independent of
  # $rc and of whether this robot's own run was clean, because a robot
  # legitimately SKIPPED for one measurement must not be read as a failure
  # to find samples; only the aggregate below decides that.
  robot_multi_kept="$(grep -oE '^multi-pair: requested [0-9]+, separated \(oracle > 0 on at least one pair\) [0-9]+, kept [0-9]+$' "$out" | grep -oE '[0-9]+$' || true)"
  robot_box_kept="$(grep -oE '^box x box: requested [0-9]+, separated \(oracle > 0 on at least one pair\) [0-9]+, kept [0-9]+$' "$out" | grep -oE '[0-9]+$' || true)"
  multi_pair_kept_total=$((multi_pair_kept_total + ${robot_multi_kept:-0}))
  box_box_kept_total=$((box_box_kept_total + ${robot_box_kept:-0}))
done

require_nonempty "${#SUMMARY[@]}" "robots to measure"

echo "=== summary ==="
printf '  %s\n' "${SUMMARY[@]}"
echo "  multi-pair samples kept across all robots: $multi_pair_kept_total"
echo "  box x box samples kept across all robots:  $box_box_kept_total"

# The guard the sibling does not need (see header): SKIPPED is a legitimate
# per-robot outcome, so a clean per-robot loop can still have scored nothing
# for one whole measurement if the link-selection filter regresses. These
# catch that where the per-robot loop above cannot.
require_nonempty "$multi_pair_kept_total" "multi-pair samples scored across all robots combined"
require_nonempty "$box_box_kept_total" "box x box samples scored across all robots combined"

if [[ "$status" -ne 0 ]]; then
  echo "FAIL Phase 3's distance clause is not met on the multi-pair / box x box penetration subpopulations." >&2
  exit 1
fi

echo "OK Phase 3's distance clause holds at 1e-4 on the multi-pair and box x box penetration subpopulations (mesh not covered -- see header)."
