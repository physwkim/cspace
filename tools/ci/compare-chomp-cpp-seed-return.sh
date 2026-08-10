#!/bin/bash
# Cross-tabulates, per problem, whether this port returned the seed
# trajectory untouched against whether upstream's C++ CHOMP returned a path
# of exactly the same length.
#
# # What each side contributes, and why neither is re-derived here
#
# The port side is a *code-level* fact, not a length argument: a record whose
# `loop.evaluations` is 1 and whose `loop.accepted` is 0 had its
# `best_group_trajectory` snapshotted on the iteration-0 pass
# (`crates/cspace-planners-chomp/src/optimizer.rs:1873`) and never replaced,
# and that snapshot is what `optimize` copies back before returning
# (`crates/cspace-planners-chomp/src/optimizer.rs:1979-1983`). So the returned
# trajectory is the seed, byte for byte, and no metric has to be trusted to
# say so.
#
# The C++ side has no such window: `chompPlan` publishes only
# `{id, solved, error_code, failure, length?, condition2...}`
# (`tools/moveit-oracle/src/oracle.cpp:6247-6259`), so the one observable is
# `length`. This script therefore compares that number to the PORT's
# `length` for the same problem rather than to an analytic seed length --
# both harnesses sum the same plan-space metric over consecutive waypoints
# (`tools/moveit-oracle/src/oracle.cpp:6021-6034` and
# `crates/cspace-planners-chomp/examples/chomp_benchmark_port.rs:627-630`,
# whose subspace weights are built by the same rule at
# `tools/moveit-oracle/src/oracle.cpp:695` and
# `crates/cspace-planners-sbp/src/joint_model_group_space.rs:153`), so
# equality between the two sides is a statement about the two paths and not
# about a metric this script re-implemented. Re-deriving the seed length from
# the problem's endpoints and the URDF bounds would be a second
# implementation of that metric, which is the failure mode a comparison
# instrument exists to avoid.
#
# What the cross-tab can and cannot say: a bitwise-equal length is not proof
# of an equal path. The `margin` line is what bounds that -- it prints the
# smallest nonzero relative length difference in the population, so a reader
# can see how far the "equal" cell sits from the nearest "unequal" one
# rather than taking the tolerance on faith.
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"

. "$REPO_ROOT/tools/ci/gate-lib.sh"

require_caller_tree "$REPO_ROOT"

usage() {
  cat >&2 <<'USAGE'
usage: compare-chomp-cpp-seed-return.sh <port_dir> <cpp_dir> [config ...]

  port_dir  a measure-chomp-objective.sh output directory
            (chomp.<config>.ndjson, with `objective` and `loop`)
  cpp_dir   a measure-phase8-cpp-baseline.sh chomp output directory
            (chomp.<config>.ndjson)
  config    configs to compare (default: floor_wall cage)
USAGE
  exit 2
}

[ $# -ge 2 ] || usage
PORT_DIR="$1"
CPP_DIR="$2"
shift 2
CONFIGS=("$@")
[ ${#CONFIGS[@]} -gt 0 ] || CONFIGS=(floor_wall cage)

for config in "${CONFIGS[@]}"; do
  port="$PORT_DIR/chomp.$config.ndjson"
  cpp="$CPP_DIR/chomp.$config.ndjson"
  for f in "$port" "$cpp"; do
    [ -s "$f" ] || { echo "FAIL $f is empty or absent" >&2; exit 1; }
  done

  # A port record without the two fields this whole cross-tab reads would
  # otherwise be counted into the "port optimized" column by default, which
  # is the same silent-reclassification the summary guards against.
  missing=$(jq -s '[.[] | select(.solved)
                   | select((has("objective") and has("loop")) | not)] | length' "$port")
  if [ "$missing" -ne 0 ]; then
    echo "FAIL $missing solved port records for $config carry no objective/loop" >&2
    exit 1
  fi

  jq -n -s --arg config "$config" \
    --slurpfile port <(jq -s '.' "$port") \
    --slurpfile cpp <(jq -s '.' "$cpp") '
    ($port[0] | map({key: (.id | tostring), value: .}) | from_entries) as $p
    | ($cpp[0] | map({key: (.id | tostring), value: .}) | from_entries) as $c
    | [$p | keys[]] as $ids
    | ($ids | map($p[.]) | map(select(.solved)) | length) as $port_solved
    | ($ids | map($c[.]) | map(select(.solved)) | length) as $cpp_solved
    | ($ids | map({p: $p[.], c: $c[.]}) | map(select(.p.solved and .c.solved))) as $both
    # `seed` on the port side is the trace, not the length: one evaluation and
    # no accepted pass means `best` is still the iteration-0 snapshot.
    | ($both | map(. + {seed: (.p.loop.evaluations == 1 and .p.loop.accepted == 0),
                        same: (.p.length == .c.length)})) as $rows
    | ($rows | map(select(.p.length != .c.length)
                   | ((.p.length - .c.length) | fabs) / .p.length)) as $devs
    | {config: $config,
       port_solved: $port_solved,
       cpp_solved: $cpp_solved,
       both_solved: ($both | length),
       port_only: ($ids | map({p: $p[.], c: $c[.]})
                   | map(select(.p.solved and (.c.solved | not))) | length),
       cpp_only: ($ids | map({p: $p[.], c: $c[.]})
                  | map(select((.p.solved | not) and .c.solved)) | length),
       port_seed_cpp_same_length:
         ($rows | map(select(.seed and .same)) | length),
       port_seed_cpp_other_length:
         ($rows | map(select(.seed and (.same | not))) | length),
       port_optimized_cpp_same_length:
         ($rows | map(select((.seed | not) and .same)) | length),
       port_optimized_cpp_other_length:
         ($rows | map(select((.seed | not) and (.same | not))) | length),
       margin_smallest_nonzero_relative_length_difference:
         (if ($devs | length) == 0 then null else ($devs | sort | .[0]) end)}
  '
done
