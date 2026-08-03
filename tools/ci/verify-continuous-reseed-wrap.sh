#!/bin/bash
# Pins the oracle's continuous-joint reseed formula (`sampleReseed`'s
# `continuous` branch in oracle.cpp) directly against wrap-vs-clamp,
# instead of through the aggregate IK-success-rate sweep
# (`run-oracle-sweep.sh`) that round 6 already showed cannot see it: at
# `--ik-consistency-limit 4.0` the branch fires on ~21% of reseed draws, and
# pr2's arm redundancy absorbs the effect before it reaches the success-rate
# statistic (see PORTING-PLAN.md's p1-joints round 6/7 sections). If
# `oracle.cpp` ever reverts that branch to clamping, nothing else in this
# repo notices; this does.
#
# Needs docker and the moveit-rs/oracle image -- like run-oracle-sweep.sh and
# verify-fixture-provenance.sh, this is deliberately not one of the
# `check-*.sh` scripts `.github/workflows/ci.yml` and the local gate loop run,
# and is not a `cargo test`. The name is the whole mechanism: ci.yml globs
# `tools/ci/check-*.sh` rather than enumerating, precisely so a new check
# cannot be forgotten -- which means a docker-requiring script named `check-*`
# is picked up by a runner that has no docker and no oracle image, and fails
# there for a reason unrelated to what it tests.
#
#   tools/ci/verify-continuous-reseed-wrap.sh
#
# # The property, and why it separates wrap from clamp
#
# The oracle's reseed always draws near the joint's bounds midpoint (0 for a
# continuous joint's [-pi, pi] reporting bounds -- ik()'s own seed_active[k]
# construction). At limit = 4.0 > pi, wrap and clamp are NOT distinguishable
# by output *range* alone: both only ever land in [-pi, pi]. They ARE
# distinguishable by *density*. Wrap draws uniformly from the unclamped
# [-limit, limit] and folds every value outside (-pi, pi] back onto the
# opposite edge, which doubles the density in the two bands adjacent to -pi
# and +pi (each of half-width limit - pi) relative to the interior -- a
# real, calculable pile-up at the boundary. Clamp instead narrows the
# *sampling interval itself* to [-pi, pi] before drawing, which is uniform
# everywhere: no pile-up, for any limit > pi.
#
# This script computes both models' predicted edge-band population fraction
# analytically from limit and asserts the oracle's observed fraction sits on
# the wrap side of their midpoint.
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$REPO_ROOT"

JOINT="r_wrist_roll_joint"   # pr2's right_arm: a real continuous revolute joint.
NEAR="0.0"
LIMIT="4.0"
COUNT="20000"

# `RESPONSE_FILE` rather than a captured shell variable: `count` draws
# serialized to JSON is large enough (a few hundred KB at this script's
# default) to blow past this host's effective ARG_MAX if passed as a single
# argv element to the analysis step below.
RESPONSE_FILE="$(mktemp)"
trap 'rm -f "$RESPONSE_FILE"' EXIT

python3 - "$JOINT" "$NEAR" "$LIMIT" "$COUNT" <<'PYEOF' | \
  "$REPO_ROOT/tools/moveit-oracle/run-oracle.sh" \
    --urdf "$REPO_ROOT/crates/moveit-kinematics/tests/fixtures/pr2.urdf" \
    --srdf "$REPO_ROOT/crates/moveit-kinematics/tests/fixtures/pr2.srdf" \
    2>/dev/null | tail -1 > "$RESPONSE_FILE"
import json, sys
joint, near, limit, count = sys.argv[1:5]
print(json.dumps({
    "id": 0,
    "op": "ik",
    "reseed_probe": {
        "joint": joint,
        "near": float(near),
        "limit": float(limit),
        "count": int(count),
    },
}))
PYEOF

python3 - "$RESPONSE_FILE" "$JOINT" "$NEAR" "$LIMIT" <<'PYEOF'
import json, math, sys

response_file, joint, near_arg, limit_arg = sys.argv[1:5]
near = float(near_arg)
limit = float(limit_arg)

with open(response_file) as f:
    response = json.load(f)

# The edge-band derivation below assumes the fold is symmetric around zero
# (near = 0), matching the oracle's own reseed, which always draws near the
# joint's bounds midpoint. A different near needs a different derivation,
# not just different numbers plugged into this one.
assert near == 0.0, "edge-band derivation assumes near=0.0, got %r" % near

if not response.get("ok"):
    print("FAIL oracle request failed: %r" % (response,), file=sys.stderr)
    sys.exit(1)

result = response["result"]
if not result["continuous"]:
    print("FAIL %r is not reported continuous -- fixture or joint name drifted, "
          "the wrap-vs-clamp property this script tests does not apply here" % joint,
          file=sys.stderr)
    sys.exit(1)

draws = result["draws"]
n = len(draws)

# Edge band: |value| > t, where t = 2*pi - limit -- the boundary of the
# region wrap folds a raw out-of-range draw back into. See this file's
# header comment for the derivation.
t = 2.0 * math.pi - limit
edge = sum(1 for v in draws if abs(v) > t)
edge_fraction = edge / n

edge_width = 2.0 * (limit - math.pi)
wrap_expected = edge_width / (2.0 * limit) * 2.0   # density 2*(1/(2*limit)) over edge_width
clamp_expected = edge_width / (2.0 * math.pi)      # flat density 1/(2*pi) over edge_width
threshold = (wrap_expected + clamp_expected) / 2.0

print("draws: %d, edge band |v| > %.6f: %d (%.4f%%)" % (n, t, edge, edge_fraction * 100))
print("wrap-predicted: %.4f%%, clamp-predicted: %.4f%%, threshold: %.4f%%" %
      (wrap_expected * 100, clamp_expected * 100, threshold * 100))

if edge_fraction <= threshold:
    print("FAIL edge-band fraction %.4f%% is at or below the wrap/clamp midpoint %.4f%% "
          "-- the continuous-joint reseed no longer wraps past the boundary; "
          "oracle.cpp likely reverted to clamping" % (edge_fraction * 100, threshold * 100),
          file=sys.stderr)
    sys.exit(1)

print("OK: edge-band fraction %.4f%% is above the wrap/clamp midpoint %.4f%% "
      "-- consistent with wrap, not clamp" % (edge_fraction * 100, threshold * 100))
PYEOF
