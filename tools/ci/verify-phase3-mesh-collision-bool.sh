#!/bin/bash
# Phase 3's `collision: bool` clause on the sub-population every sibling in
# this directory excludes by construction: pairs where EITHER side is a mesh.
#
# `tools/moveit-diff/src/bin/mesh_collision_bool_ladder.rs` was built to
# measure exactly this population -- see that file's own module doc for the
# full method (prbt, a synthetic unit-cube mesh reused from
# `exact_tangency_is_decided_per_shape_pair.rs`, an 11-decade positive/
# negative gap ladder plus exact tangency, every shape pair from the 5x5
# `{box, sphere, cylinder, cone, mesh}` grid the oracle's wire protocol can
# build). Until this script it had no CI gate and no recorded verdict
# anywhere in this repository: `git log -S 'mesh_collision_bool_ladder'
# -- tools/ci` returns nothing before this commit. This script is that gate,
# and -- like `verify-phase3-penetration-extended.sh` -- it has never itself
# been run; see "State of this gate" below.
#
# # Why the binary, not a shell loop over the other siblings
#
# `verify-phase3-collision-sweep.sh`, `-tangency-subset.sh` and
# `-penetration-subset.sh` all target robots where "every link is a single
# mesh" is exactly the population they exclude (see each script's own
# header). None of their corpora can be pointed at a mesh pair without
# becoming a different measurement; `mesh_collision_bool_ladder` exists
# because nothing else in this tree puts a mesh cell in front of the oracle
# at all.
#
# # What stays OUT: cone x anything, against the oracle
#
# `oracle.cpp`'s `parseShape` has no `"cone"` branch (a wire-protocol
# ceiling, not a bug -- see the binary's own module doc), so the 9 of 25
# grid cells involving `cone` are reported from this port alone, each row
# printed with `"oracle": null` and never counted toward `mismatches`. A
# pass below says nothing about cone x {box, sphere, cylinder, cone, mesh}.
#
# # Robot
#
# prbt only, per the binary's own construction (`prbt_base_link`'s default
# state world transform is the identity, which is what lets an attached
# shape's pose stand in for its world pose without a second transform this
# script would have to get right). The binary takes no robot argument --
# unlike its siblings, this measures one fixed grid on one fixture, not a
# roster.
#
# # State of this gate: UNEXECUTED
#
# Written and reviewed, never run -- same state `verify-phase3-penetration-
# extended.sh` documents for itself and for the same reason: no measured
# wall clock is guessed at here. `verify-mpr-vs-epa.sh`'s own cost note (23
# ladder points x up to 16 non-cone-excluded shape pairs each, one oracle
# round trip per scored cell) suggests low hundreds of oracle requests, one
# process, no sharding -- comparable to `verify-phase3-tangency-subset.sh`'s
# prbt row (4-5s) rather than to its pr2 row, but that is an estimate from
# the grid's own shape, not a measurement.
#
# Needs docker (through `sg`, per this repo's wrapper rule) and the
# digest-gated oracle image. Without them it SKIPs loudly: a silent skip is
# indistinguishable from a pass.
#
#   sg docker -c tools/ci/verify-phase3-mesh-collision-bool.sh
set -uo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"

. "$REPO_ROOT/tools/ci/gate-lib.sh"
require_caller_tree "$REPO_ROOT"
# This script runs without -e on purpose, so a failed cd would not abort it
# and every path below would resolve against the caller's directory instead.
cd "$REPO_ROOT" || exit 1

BIN="$REPO_ROOT/target/release/mesh_collision_bool_ladder"

if ! command -v docker >/dev/null 2>&1; then
  skip_not_measured blocked \
    "docker is not on PATH -- the mesh-pair penetration/gap population is not measured by this run." \
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

# Release, not debug: each of the grid's cells is its own oracle round trip,
# so an unoptimised build makes this side, not the oracle, the bottleneck --
# the same reasoning as every sibling's own release build.
if ! cargo build --release --manifest-path "$REPO_ROOT/Cargo.toml" -p moveit-diff \
  --bin mesh_collision_bool_ladder; then
  echo "FAIL could not build mesh_collision_bool_ladder" >&2
  exit 1
fi

OUT_DIR="$(mktemp -d)"
trap 'rm -rf "$OUT_DIR"' EXIT

echo
echo "=== Phase 3 collision clause, mesh-pair population ==="
echo "    prbt, synthetic unit-cube mesh vs the 5x5 {box, sphere, cylinder,"
echo "    cone, mesh} grid the oracle's wire protocol can build (cone x cone"
echo "    and every cone pair reported port-only, never scored -- see header)"
echo

out="$OUT_DIR/prbt.out"

# Redirected to a file, never piped: a pipeline reports the filter's status,
# which is how a disagreement becomes a silent pass.
start="$SECONDS"
"$BIN" \
  --urdf "$REPO_ROOT/fixtures/prbt.urdf" \
  --srdf "$REPO_ROOT/fixtures/prbt.srdf" \
  --oracle "$REPO_ROOT/tools/moveit-oracle/run-oracle.sh" \
  > "$out" 2>&1
rc=$?
elapsed=$((SECONDS - start))

grep -vE '^\[(WARN|INFO|ERROR)\]' "$out"
echo "wall clock: ${elapsed}s"
echo

# The binary's own summary line is its verdict marker: reaching it means the
# run completed the whole grid rather than being killed mid-flight. See
# `run_verdict` -- a nonzero exit without this line is reported as
# "incomplete", not conflated with a real mismatch.
verdict="$(run_verdict "$rc" "$out" '^[0-9]+ cells, [0-9]+ scored against the oracle, [0-9]+ mismatch')"
case "$verdict" in
  ok)
    echo "=== summary ==="
    echo "  prbt: MET (${elapsed}s)"
    echo "OK Phase 3's collision clause holds on the mesh-pair population (cone excluded -- see header)."
    ;;
  disagreed)
    echo "=== summary ==="
    echo "  prbt: NOT MET (${elapsed}s)"
    echo "FAIL Phase 3's collision clause is not met on the mesh-pair population." >&2
    exit 1
    ;;
  *)
    echo "=== summary ==="
    echo "  prbt: $verdict (${elapsed}s)"
    echo "FAIL prbt: $verdict" >&2
    exit 1
    ;;
esac
