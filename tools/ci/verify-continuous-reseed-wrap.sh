#!/bin/bash
# Pins the oracle's continuous-joint reseed formula (`sampleReseed`'s
# `continuous` branch in oracle.cpp) directly against wrap-vs-clamp,
# instead of through the aggregate IK-success-rate sweep
# (`verify-oracle-sweep.sh`) that round 6 already showed cannot see it: at
# `--ik-consistency-limit 4.0` the branch fires on ~21% of reseed draws, and
# pr2's arm redundancy absorbs the effect before it reaches the success-rate
# statistic (see PORTING-PLAN.md's p1-joints round 6/7 sections). If
# `oracle.cpp` ever reverts that branch to clamping, nothing else in this
# repo notices; this does.
#
# Needs docker and the moveit-rs/oracle image -- like verify-oracle-sweep.sh and
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

. "$REPO_ROOT/tools/ci/gate-lib.sh"

require_caller_tree "$REPO_ROOT"
cd "$REPO_ROOT"

JOINT="r_wrist_roll_joint"   # pr2's right_arm: a real continuous revolute joint.
NEAR="0.0"
LIMIT="4.0"
COUNT="20000"

# `RESPONSE_FILE` rather than a captured shell variable: `count` draws
# serialized to JSON is large enough (a few hundred KB at this script's
# default) to blow past this host's effective ARG_MAX if passed as a single
# argv element to the analysis step below.
REQUEST_FILE="$(mktemp)"
RAW_FILE="$(mktemp)"
STDERR_FILE="$(mktemp)"
RESPONSE_FILE="$(mktemp)"
trap 'rm -f "$REQUEST_FILE" "$RAW_FILE" "$STDERR_FILE" "$RESPONSE_FILE"' EXIT

python3 - "$JOINT" "$NEAR" "$LIMIT" "$COUNT" > "$REQUEST_FILE" <<'PYEOF'
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

# Not `python3 ... | run-oracle.sh ... 2>/dev/null | tail -1`, which is how
# this was first written and which had both of the failure modes
# `verify-oracle-sweep.sh`'s own comment warns about. The pipe made `tail` the
# status-reporting stage, and `2>/dev/null` threw away the only text that says
# what went wrong -- so on a host where the caller lacks the docker group, this
# script exited 1 having printed nothing at all, which reads as "the check is
# broken" rather than "docker is unreachable". stderr is captured instead of
# discarded (the oracle's build chatter is noise on the success path, evidence
# on the failure path) and replayed only when the run fails.
status=0
"$REPO_ROOT/tools/moveit-oracle/run-oracle.sh" \
  --urdf "$REPO_ROOT/crates/cspace-core/tests/fixtures/kinematics/pr2.urdf" \
  --srdf "$REPO_ROOT/crates/cspace-core/tests/fixtures/kinematics/pr2.srdf" \
  < "$REQUEST_FILE" > "$RAW_FILE" 2> "$STDERR_FILE" || status=$?

if [[ $status -ne 0 ]]; then
  echo "run-oracle.sh failed (exit $status). Its stderr:" >&2
  cat "$STDERR_FILE" >&2
  exit "$status"
fi

# The oracle emits one JSON object per line; the last is this script's only
# request. Split from the run above so a malformed response is diagnosed here
# rather than silently becoming an empty file.
tail -1 "$RAW_FILE" > "$RESPONSE_FILE"
if [[ ! -s "$RESPONSE_FILE" ]]; then
  echo "run-oracle.sh exited 0 but produced no response line. Its stderr:" >&2
  cat "$STDERR_FILE" >&2
  exit 1
fi

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
