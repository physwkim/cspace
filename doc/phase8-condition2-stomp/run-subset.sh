#!/bin/bash
# Re-runs a named id subset with the SAME construction the sweep used:
# measure-phase8-condition2-grid.sh gives STOMP PROCS=N so PER=1, i.e. one
# process per problem whose .in holds exactly that problem, and the harness
# seeds from seed_base.wrapping_add(problem.id). Both make a single-problem
# re-run bit-for-bit the same computation as its shard in the full sweep.
set -euo pipefail
BIN="$1"; IN_JSON="$2"; WORK="$3"; shift 3
rm -rf "$WORK"; mkdir -p "$WORK"
pids=()
for id in "$@"; do
  jq --argjson lo "$id" --argjson hi "$((id + 1))" '.problems |= .[$lo:$hi]' \
    "$IN_JSON" > "$WORK/$id.in"
  "$BIN" 700001 1e9 <"$WORK/$id.in" >"$WORK/$id.out" 2>"$WORK/$id.err" &
  pids+=($!)
done
fail=0
for p in "${pids[@]}"; do wait "$p" || fail=1; done
[ "$fail" -eq 0 ] || { echo "a subset shard exited nonzero" >&2; exit 1; }
cat "$WORK"/*.out | jq -s -c 'sort_by(.id) | .[]'
