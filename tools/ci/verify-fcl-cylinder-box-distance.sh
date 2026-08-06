#!/bin/bash
# Re-derives, against a *closed form*, how wrong the reference's published
# separation distance is on the one shape pair where this workspace has a
# disagreement it cannot arbitrate by comparison.
#
# `tools/ci/verify-fcl-distance-tolerance.sh` already measures that fcl's own
# answer moves when only the GJK stopping threshold changes. That is enough to
# say the reference is imprecise and not enough to say *which* answer is right:
# both of its columns are fcl's. This script closes that gap by picking a
# configuration whose correct answer needs no narrowphase at all -- a cylinder
# above the top face of `tools/moveit-diff`'s floor box -- so every column can
# be scored against the truth rather than against another column.
#
# Four checks, and they fail in different directions so one cannot be relaxed
# to rescue another:
#
#   1. at MoveIt's default `distance_tolerance` (`1e-6`, the value
#      `fcl::DistanceRequestd(cdata->req->enable_nearest_points)` leaves in
#      place at
#      `moveit_core/collision_detection_fcl/src/collision_common.cpp:603`) fcl's
#      answer is off the closed form by at least `MIN_DEFAULT_ERROR` somewhere
#      in the sample -- i.e. the published number is wrong by more than
#      PORTING-PLAN.md §5 Phase 3's own `1e-4`;
#   2. tightening only that threshold brings the whole sample inside
#      `MAX_TIGHT_ERROR`, so check 1 is the threshold's doing and not a wrong
#      algorithm or a wrong closed form;
#   3. `GST_INDEP` at the same tightened threshold -- a different algorithm --
#      lands inside `MAX_INDEP_ERROR` of the same closed form, so "the closed
#      form is right" does not rest on libccd agreeing with itself;
#   4. swapping the two operands moves the default-threshold answer by at least
#      `MIN_ORDER_SPREAD`. This is the check that makes the defect visible as a
#      defect rather than as noise: the exact answer cannot depend on argument
#      order, so a spread this size is the reference contradicting itself.
#
# Plus one pin: pose 0 of the probe is case 8148 of the seed-1 prbt sweep, and
# its box-first column must still reproduce the value the oracle published for
# that case, bit for bit. Without it the four bounds above could all pass on a
# sample that no longer contains the case this script exists for.
#
# Needs docker (via `sg`, per this repo's wrapper rule) and the digest-gated
# oracle image, for the same reason `verify-fcl-distance-tolerance.sh` does:
# the fcl that matters is the one the oracle links, not a host build. Without
# them it SKIPs loudly -- a silent skip is indistinguishable from a pass.
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"

. "$REPO_ROOT/tools/ci/gate-lib.sh"

require_caller_tree "$REPO_ROOT"
cd "$REPO_ROOT"

PROBE="$REPO_ROOT/tools/fcl-cylinder-box-distance-probe/probe.cpp"

# Every bound was measured before it was set, by running the probe and reading
# its printed maxima; none is copied from a neighbouring check. First run
# inside `moveit-rs/oracle:fc6738ad78dd45d5` (`libfcl-dev 0.7.0-3build2`),
# 2000 poses, error against the closed form:
#
#   default `1e-6`, cylinder first : max 2.513565e-03 (5 poses over 1e-4)
#   default `1e-6`, box first      : max 1.683122e-03 (5 poses over 1e-4)
#   tightened `1e-12`              : max 1.335130e-09
#   tightened `1e-12` + GST_INDEP  : max 6.287026e-12
#   |box first - cylinder first|   : max 2.513557e-03 (10 poses over 1e-4)
#
# `MIN_DEFAULT_ERROR` and `MIN_ORDER_SPREAD` are floors on a defect (assert it
# is at least this bad); `MAX_TIGHT_ERROR` and `MAX_INDEP_ERROR` are ceilings on
# agreement. Both floors are set at `1e-4` rather than just under the
# measurement, because `1e-4` is the number the claim is about -- Phase 3's own
# distance tolerance -- and it is the loosest floor that still says "the
# reference misses by more than the clause allows". The ceilings carry a margin
# over the measurement: `1e-8` is 7.5x the tightened max, `5e-11` is 8.0x the
# indep max.
MIN_DEFAULT_ERROR="1e-4"
MIN_ORDER_SPREAD="1e-4"
MAX_TIGHT_ERROR="1e-8"
MAX_INDEP_ERROR="5e-11"
# What the oracle published for `floor(world_object)/prbt_flange(robot_link)` at
# case 8148 of `--cases 10000 --seed 1 --floor-top-z -0.5`, through MoveIt's own
# `CollisionEnvFCL::distanceRobot`. The probe's box-first column reproduces it
# from bare `fcl::distance`, which is what identifies the wrapper as innocent.
CASE_8148_ORACLE="3.11769210552093334e-1"

if ! command -v docker >/dev/null 2>&1; then
  echo "SKIP docker not on PATH -- fcl's cylinder/box distance is not re-measured."
  echo "SKIP this is not a pass."
  exit 0
fi

# shellcheck source=tools/moveit-oracle/src-digest.sh
source "$REPO_ROOT/tools/moveit-oracle/src-digest.sh"
want="$(oracle_stamp "$REPO_ROOT/tools/moveit-oracle")"
IMAGE="${IMAGE:-$(oracle_image_tag "$want")}"

have="$(docker run --rm --entrypoint cat "$IMAGE" /usr/local/share/oracle-src.sha256 2>/dev/null || true)"
if [[ "$have" != "$want" ]]; then
  echo "SKIP $IMAGE is missing or built from different oracle sources"
  echo "SKIP   image: ${have:-<missing or unstamped>}"
  echo "SKIP   tree:  $want"
  echo "SKIP this is not a pass; rebuild with tools/moveit-oracle/build.sh, and remember"
  echo "SKIP that an unwrapped docker call here reports failure as success -- use sg docker."
  exit 0
fi

work="$(mktemp -d)"
trap 'rm -rf "$work"' EXIT
cp "$PROBE" "$work/probe.cpp"

start=$SECONDS
docker run --rm -v "$work:$work" -w "$work" --entrypoint bash "$IMAGE" -c '
  set -euo pipefail
  # fcl/common/types.h includes <Eigen/Dense> unqualified, so Eigen needs its
  # own include dir on the path -- the image has it under eigen3/.
  g++ -O2 -std=c++17 $(pkg-config --cflags eigen3) -o probe probe.cpp -lfcl -lccd 2>&1
  ./probe > poses.csv
  dpkg-query -W libfcl-dev > fclver.txt 2>/dev/null || echo "libfcl-dev <unknown>" > fclver.txt
' >"$work/build.log" 2>&1 || { echo "FAIL probe build/run failed inside $IMAGE:"; sed 's/^/  | /' "$work/build.log"; exit 1; }
elapsed=$((SECONDS - start))

echo "== $(cat "$work/fclver.txt") in $IMAGE, ${elapsed}s"

MIN_DEFAULT_ERROR="$MIN_DEFAULT_ERROR" MIN_ORDER_SPREAD="$MIN_ORDER_SPREAD" \
  MAX_TIGHT_ERROR="$MAX_TIGHT_ERROR" MAX_INDEP_ERROR="$MAX_INDEP_ERROR" \
  CASE_8148_ORACLE="$CASE_8148_ORACLE" \
  python3 - "$work/poses.csv" <<'PY'
import csv, os, sys

rows = list(csv.DictReader(open(sys.argv[1])))
if not rows:
    sys.exit("FAIL probe emitted no admissible poses")

min_default = float(os.environ["MIN_DEFAULT_ERROR"])
min_spread = float(os.environ["MIN_ORDER_SPREAD"])
max_tight = float(os.environ["MAX_TIGHT_ERROR"])
max_indep = float(os.environ["MAX_INDEP_ERROR"])
case_8148 = float(os.environ["CASE_8148_ORACLE"])


def err(column):
    """|column - closed form| per pose, worst first."""
    e = [(abs(float(r[column]) - float(r["closed"])), int(r["idx"])) for r in rows]
    e.sort(reverse=True)
    return e


cyl_first = err("d_cyl_first")
box_first = err("d_box_first")
tight = err("d_tight")
indep = err("d_indep")
spread = sorted(((abs(float(r["d_box_first"]) - float(r["d_cyl_first"])), int(r["idx"]))
                 for r in rows), reverse=True)
default_worst = max(cyl_first[0], box_first[0])

over = lambda e, b: sum(1 for d, _ in e if d > b)
print(f"{len(rows)} admissible cylinder/box poses, error against the closed form")
print(f"  default 1e-6, cylinder first : max {cyl_first[0][0]:.6e} (pose {cyl_first[0][1]}), "
      f">1e-4 {over(cyl_first, 1e-4)}")
print(f"  default 1e-6, box first      : max {box_first[0][0]:.6e} (pose {box_first[0][1]}), "
      f">1e-4 {over(box_first, 1e-4)}")
print(f"  tightened 1e-12              : max {tight[0][0]:.6e} (pose {tight[0][1]})")
print(f"  tightened 1e-12 + GST_INDEP  : max {indep[0][0]:.6e} (pose {indep[0][1]})")
print(f"  |box first - cylinder first| : max {spread[0][0]:.6e} (pose {spread[0][1]}), "
      f">1e-4 {over(spread, 1e-4)}")

bad = []
if default_worst[0] < min_default:
    bad.append(f"FAIL at MoveIt's default distance_tolerance fcl misses the closed form by at "
               f"most {default_worst[0]:.6e}, under the pinned {min_default:.0e}: the reference "
               f"is no longer wrong by more than Phase 3's own tolerance, so the claim this "
               f"check supports is stale")
if tight[0][0] > max_tight:
    bad.append(f"FAIL tightening distance_tolerance still leaves {tight[0][0]:.6e} of error "
               f"(pose {tight[0][1]}), over the pinned {max_tight:.0e}: check 1 cannot be read "
               f"as the stopping threshold's doing")
if indep[0][0] > max_indep:
    bad.append(f"FAIL GST_INDEP misses the same closed form by {indep[0][0]:.6e} "
               f"(pose {indep[0][1]}), over the pinned {max_indep:.0e}: two independent "
               f"algorithms disagreeing with it means the closed form is what is wrong")
if spread[0][0] < min_spread:
    bad.append(f"FAIL swapping the operands moves the published distance by at most "
               f"{spread[0][0]:.6e}, under the pinned {min_spread:.0e}: the reference no longer "
               f"contradicts itself on argument order, which is the defect this check names")

pose0 = rows[0]
if int(pose0["idx"]) != 0:
    bad.append("FAIL pose 0 is missing from the sample; it is the pinned case 8148 row")
elif float(pose0["d_box_first"]) != case_8148:
    bad.append(f"FAIL pose 0's box-first answer is {float(pose0['d_box_first']):.17e}, not the "
               f"{case_8148:.17e} the oracle published for case 8148: the probe is no longer "
               f"measuring the configuration this check exists for")

if bad:
    print("\n".join(bad))
    sys.exit(1)

print(f"OK fcl's published answer misses the exact one by up to {default_worst[0]:.6e} "
      f"(>= {min_default:.0e}) at MoveIt's default threshold,")
print(f"OK by {tight[0][0]:.6e} (<= {max_tight:.0e}) tightened and {indep[0][0]:.6e} "
      f"(<= {max_indep:.0e}) tightened under GST_INDEP,")
print(f"OK and moves {spread[0][0]:.6e} (>= {min_spread:.0e}) on operand order alone; "
      f"case 8148 still reproduces {case_8148:.17e}")
PY
