#!/bin/bash
# Re-derives how much of the `distance: f64` clause's own `1e-4` tolerance is
# spent by the *reference*, on the branch where the port is answerable for the
# number.
#
# `PORTING-PLAN.md` §5 Phase 3's second clause compares this port against the
# oracle's published `distance` within `1e-4`. Above
# `moveit_core/collision_detection_fcl/src/collision_common.cpp:636` that
# published value is `fcl::distance`'s own return, and `distanceCallback` asks
# for it with `fcl::DistanceRequestd(cdata->req->enable_nearest_points)`
# (`:603`) -- one argument, so `distance_tolerance` keeps its constructor
# default of `1e-6` (`fcl/narrowphase/distance_request.h`). That field is
# documented as "the threshold used in GJK algorithm to stop distance
# iteration": a progress threshold, not an error bound.
#
# The claim this script exists to keep honest is that the separated branch's
# residual disagreement in the sweep (prbt, worst `8.9e-5` on
# `prbt_link_4`/`prbt_base_link`) is inside the reference's own precision, so
# it cannot be read as a port error. Two checks, and they are independent:
#
#   1. tightening ONLY `distance_tolerance` moves fcl's own answer by at least
#      `1e-5` somewhere in the sample -- i.e. the default-tolerance answer is
#      not accurate to the scale the clause measures at;
#   2. the two tightened solvers (`GST_LIBCCD` and `GST_INDEP`, different
#      algorithms) agree far more closely than check 1's drift, so "tight" is
#      a reference rather than a third opinion. Without this, check 1 could be
#      one solver wandering. Both are tightened by the same request field:
#      `fcl/narrowphase/distance-inl.h:208` feeds it to libccd's
#      `distance_tolerance` and `:214` to indep's `gjk_tolerance`.
#
# Needs docker (via `sg`, per this repo's wrapper rule) and the digest-gated
# oracle image, for the same reason `verify-fcl-tangency-dispatch.sh` does:
# the fcl that matters is the one the oracle links, not a host build. Without
# them it SKIPs loudly -- a silent skip is indistinguishable from a pass.
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$REPO_ROOT"

PROBE="$REPO_ROOT/tools/fcl-distance-tolerance-probe/probe.cpp"

# Both bounds were measured before they were set, by running the probe and
# reading its printed maxima; neither is copied from a neighbouring check.
# `MIN_DRIFT` is a floor on a defect (assert it is at least this bad) and
# `MAX_SOLVER_GAP` is a ceiling on agreement, so they fail in opposite
# directions and one cannot be relaxed to rescue the other.
#
# First run inside `moveit-rs/oracle:bf084112fdd5730b` (`libfcl-dev
# 0.7.0-3build2`), 2000 poses: `|default - tight|` max `4.418002e-04`,
# `|tight - indep|` max `9.761567e-08`. `MIN_DRIFT` is set two orders under
# the measured drift rather than just under it, because the claim it guards
# is "the default is imprecise at the scale the clause measures at" (`1e-4`)
# and `1e-5` is the loosest floor that still says so. `MAX_SOLVER_GAP` is the
# measured `9.76e-8` with a 5x margin. `MIN_RATIO` is the fact the two
# together are for: the drift is not the solvers disagreeing.
MIN_DRIFT="1e-5"
MAX_SOLVER_GAP="5e-7"
MIN_RATIO="100"

if ! command -v docker >/dev/null 2>&1; then
  echo "SKIP docker not on PATH -- fcl's default distance_tolerance is not re-measured."
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

MIN_DRIFT="$MIN_DRIFT" MAX_SOLVER_GAP="$MAX_SOLVER_GAP" MIN_RATIO="$MIN_RATIO" \
  python3 - "$work/poses.csv" <<'PY'
import csv, os, sys

rows = list(csv.DictReader(open(sys.argv[1])))
if not rows:
    sys.exit("FAIL probe emitted no separated poses")

min_drift = float(os.environ["MIN_DRIFT"])
max_gap = float(os.environ["MAX_SOLVER_GAP"])
min_ratio = float(os.environ["MIN_RATIO"])

drift = [(abs(float(r["d_default"]) - float(r["d_tight"])), int(r["idx"])) for r in rows]
gap = [(abs(float(r["d_tight"]) - float(r["d_indep"])), int(r["idx"])) for r in rows]
drift.sort(reverse=True)
gap.sort(reverse=True)

over = {b: sum(1 for d, _ in drift if d > b) for b in (1e-6, 1e-5, 1e-4)}
print(f"{len(rows)} separated box/cylinder poses")
print(f"  |default - tight| : max {drift[0][0]:.6e} (pose {drift[0][1]}), "
      f">1e-6 {over[1e-6]}, >1e-5 {over[1e-5]}, >1e-4 {over[1e-4]}")
ratio = drift[0][0] / gap[0][0] if gap[0][0] > 0 else float("inf")
print(f"  |tight   - indep| : max {gap[0][0]:.6e} (pose {gap[0][1]}), "
      f"{ratio:.0f}x under the drift above")

bad = []
if drift[0][0] < min_drift:
    bad.append(f"FAIL tightening distance_tolerance moved fcl's answer by at most "
               f"{drift[0][0]:.6e}, under the pinned {min_drift:.0e}: the default is no longer "
               f"imprecise at the scale this check claims, so the claim it supports is stale")
if gap[0][0] > max_gap:
    bad.append(f"FAIL the two tightened solvers differ by {gap[0][0]:.6e} (pose {gap[0][1]}), "
               f"over the pinned {max_gap:.0e}: 'tight' is not a reference here and check 1 "
               f"cannot be read as the default's error")
if ratio < min_ratio:
    bad.append(f"FAIL the drift is only {ratio:.1f}x the inter-solver spread, under the pinned "
               f"{min_ratio:.0f}x: check 1's movement is not distinguishable from the two "
               f"algorithms simply disagreeing")
if bad:
    print("\n".join(bad))
    sys.exit(1)

print(f"OK fcl's own answer moves up to {drift[0][0]:.6e} on tolerance alone (>= {min_drift:.0e}),")
print(f"OK while its two tightened solvers agree to {gap[0][0]:.6e} (<= {max_gap:.0e}, "
      f"{ratio:.0f}x under)")
PY
