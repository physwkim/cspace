#!/bin/bash
# tools/ci/requirement-message-closure.py -- RUNS INSIDE THE ORACLE
# CONTAINER, per its own module doc -- had no caller anywhere in this
# repository before this script: `rg -n requirement-message-closure`
# outside this file and its own docstring returns nothing, and the exact
# docker invocation its header documents only ever existed as prose for a
# person to type by hand. This is that invocation, made a gate.
#
# # Two things the target script's own docstring gets wrong
#
# Its module doc calls itself `measure-requirement-closure.py` three times
# and describes a `--client-messages` flag for a container-free, port-side-
# only run. Neither is true of the file on disk: the file is
# `requirement-message-closure.py` (`git log -- tools/ci/requirement-
# message-closure.py` shows one commit, c71ab038, and no rename), it takes
# no arguments at all (`rg -n 'argparse|sys\.argv' tools/ci/requirement-
# message-closure.py` is empty), and `main()` unconditionally requires
# `/ws/install/*/share` or `/opt/ros/*/share` to be non-empty -- there is no
# path through this script that runs without the container. Left uncorrected
# here deliberately: fixing another script's docstring is outside what this
# commit is for (wiring the orphan), and the wrapper below does not rely on
# the missing flag -- it runs the one mode that actually exists.
#
# # What this gate actually checks, and what it cannot
#
# The script's own `main()` returns 1 only when the ROS share/ tree is
# missing (wrong host) -- it does NOT fail when `missing` (unresolved type
# definitions) is non-empty; that list is printed, not asserted on. So a
# bare `python3 requirement-message-closure.py` exiting 0 inside the
# container is not by itself evidence the closure is complete. What this
# script checks instead: the moveit_msgs half of today's closure, diffed
# against the transcript already committed at `tools/ci/requirement-
# closure-moveit-msgs.txt` (see that file's own header -- produced by this
# exact invocation, "34 types" at time of writing) -- and it FAILs loudly if
# today's run reports ANY unresolved type, since the committed transcript
# records zero.
#
# The other 30 types the header's "64 types over 9 packages" line implies
# (everything outside moveit_msgs -- std_msgs and whatever else the
# transitive walk reaches) are not checked into this repository anywhere,
# so this gate cannot detect drift in that portion; only the moveit_msgs
# subset has a committed baseline to diff against.
#
# # State of this gate: UNEXECUTED
#
# Written and reviewed, never run -- same state `verify-phase3-penetration-
# extended.sh` and `verify-phase3-mesh-collision-bool.sh` document for
# themselves. No wall clock is guessed at here; the operation is one
# container run of a pure-Python script with no oracle round trips, so it
# should be fast, but that is unmeasured.
#
# Needs docker (through `sg`, per this repo's wrapper rule) and the
# digest-gated oracle image. Without them it SKIPs loudly: a silent skip is
# indistinguishable from a pass.
#
#   sg docker -c tools/ci/verify-requirement-message-closure.sh
set -uo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"

. "$REPO_ROOT/tools/ci/gate-lib.sh"
require_caller_tree "$REPO_ROOT"
# This script runs without -e on purpose, so a failed cd would not abort it
# and every path below would resolve against the caller's directory instead.
cd "$REPO_ROOT" || exit 1

TARGET="tools/ci/requirement-message-closure.py"
BASELINE="tools/ci/requirement-closure-moveit-msgs.txt"

if ! command -v docker >/dev/null 2>&1; then
  skip_not_measured blocked \
    "docker is not on PATH -- the requirement-message closure is not measured by this run." \
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
  skip_not_measured blocked "this is not a pass -- the oracle image was never consulted."
fi

OUT_DIR="$(mktemp -d)"
trap 'rm -rf "$OUT_DIR"' EXIT
out="$OUT_DIR/closure.out"

echo
echo "=== requirement-anchored message closure, moveit_msgs half ==="
echo "    running $TARGET inside $IMAGE"
echo

# Redirected to a file, never piped: a pipeline reports the filter's status,
# which is how a run that never reached the container becomes a silent pass.
sg docker -c "docker run --rm --entrypoint bash \
    -v $REPO_ROOT/$TARGET:/tmp/rmc.py:ro \
    $IMAGE -lc 'python3 /tmp/rmc.py'" >"$out" 2>&1
rc=$?

cat "$out"
echo

if [ "$rc" -ne 0 ]; then
  echo "FAIL $TARGET exited $rc inside the container -- see output above." >&2
  exit 1
fi

# The script prints the moveit_msgs subset as one bare type name per line
# between its own two markers; extract exactly that span, nothing either
# side of it.
got_names="$OUT_DIR/got-moveit-msgs.txt"
sed -n '/^--- moveit_msgs types in the closure ---$/,/^--- unresolved\|^$/{
  /^---/d
  /^$/d
  p
}' "$out" | sort >"$got_names"

want_names="$OUT_DIR/want-moveit-msgs.txt"
grep -v '^#' "$BASELINE" | grep -v '^$' | sort >"$want_names"

fail=0

if ! diff -u "$want_names" "$got_names" >"$OUT_DIR/names.diff"; then
  echo "FAIL moveit_msgs closure does not match $BASELINE:" >&2
  cat "$OUT_DIR/names.diff" >&2
  fail=1
else
  echo "ok   moveit_msgs closure matches $BASELINE ($(wc -l <"$want_names" | tr -d ' ') types)"
fi

if grep -q '^--- unresolved' "$out"; then
  echo "FAIL closure reports unresolved type(s) -- the committed baseline records none:" >&2
  sed -n '/^--- unresolved/,$p' "$out" >&2
  fail=1
else
  echo "ok   no unresolved types reported"
fi

echo
if [ "$fail" -ne 0 ]; then
  echo "FAIL requirement-message closure drifted from its committed baseline -- see above." >&2
  exit 1
fi

echo "OK requirement-message closure (moveit_msgs half) matches its committed baseline."
