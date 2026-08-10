#!/bin/bash
# Runs upstream's own C++ CHOMP or STOMP over one Phase 8 benchmark config,
# one oracle process per problem, and writes the per-problem results as NDJSON.
#
# This is the C++ side of the comparison PORTING-PLAN.md's Phase 8 section
# needs. The port side is `chomp_benchmark_port.rs` / `stomp_benchmark_port.rs`;
# both consume the same request emitted by `plan_benchmark_problem_set`, so the
# two sides see byte-identical problems and differ only in which
# implementation solves them.
#
# # Why one process per problem
#
# Both C++ planners draw every random number from `rsl::rng()`, a
# `thread_local std::mt19937` that can be seeded exactly once per thread --
# `rsl::rng` throws on any later call carrying a seed sequence. So a seed is a
# property of a process, not of a request, and the oracle takes it as the
# startup argument `--planner-rng-seed` (see `oracle.cpp`'s `main`, which
# applies it before any `RobotModel` exists because `moveit::getLogger()`
# otherwise takes that one seedable call for itself).
#
# Running a whole config in one process would therefore make every problem's
# result depend on the entire stream of problems before it: reproducible only
# as a whole, and silently changed by any sharding. One process per problem at
# `--planner-rng-seed $((PLANNER_SEED_BASE + id))` instead mirrors the port
# harnesses exactly (both seed `seed_base.wrapping_add(problem.id)` per
# problem), which is what makes a per-problem comparison between the two sides
# meaningful and what makes this script safe to shard.
#
# The seed matters -- this is measured, not assumed. On `cage` ids 0..9 at
# `max_iterations=50`, C++ CHOMP solves 7/10 at seed base 700001 and 6/10 at
# 424242, disagreeing on ids 6 and 8. A success rate from either side is one
# draw from a seed lottery, and the tables built on this script say so.
#
# # The clock
#
# Both planners' upstream defaults stop on a wall clock -- CHOMP's
# `planning_time_limit_` (6.0s from `ChompParameters`'s constructor, 10.0s from
# `CHOMPInterface::loadParams`) and STOMP's `allowed_planning_time` (5.0s from
# `move_group`) -- which makes a success rate measured at them a property of
# how loaded this machine was rather than of the planner. PORTING-PLAN.md's
# Phase 8 section records that trap being sprung on the port side.
#
# So the clock here is set far above any observed per-problem time and the
# iteration bound (CHOMP `max_iterations`, STOMP `num_iterations`, both left at
# upstream defaults) is what terminates. It is set to a finite value rather
# than infinity on purpose: an unbounded run cannot distinguish "still working"
# from "hung", and upstream reports a clock stop as its own error code
# (`TIMED_OUT` for STOMP). The summary below counts those codes, so a run that
# quietly became clock-bounded says so in its own output instead of being
# reported as a planner failure.
#
# # Which robot
#
# From the set file's own `robot` field, never from a constant here. The
# generator takes a robot argument and writes what it used into the set
# (`plan_benchmark_problem_set.rs`'s `robot` and `group` keys), and the oracle
# is started with a `--urdf`/`--srdf` pair that has to be the same robot or the
# request's `group` does not exist in the model. This file used to hard-code
# panda's fixtures while consuming a set that names its own robot, so a fanuc
# set would have been planned against panda's model -- two sources of truth for
# one fact, with nothing comparing them. Reading it from the set is what makes
# `measure-phase8-optimizer-properties.sh`'s fanuc strata measurable at all.
#
# # The result cache
#
# A per-problem result is reused when it is already on disk, so an interrupted
# run resumes instead of re-solving 500 problems. That makes the cache's key a
# correctness question, not a bookkeeping one: a key that omits an input
# answers the second run with the first run's numbers, unread.
#
# The key is therefore not a list of variables. It is a digest of what the
# oracle process is actually given -- the request bytes on its stdin, the seed
# in its `--planner-rng-seed`, the URDF and SRDF bytes it is started with, and
# `oracle_stamp`, the digest run-oracle.sh already requires the image to carry.
# Everything that shapes a request reaches the key by being in those bytes, so
# a new request-shaping option (the next `CONDITION2_RESOLUTIONS`, the next
# clock bound) cannot be forgotten: there is no list to add it to. A name-based
# key had already lost `PLANNER_SEED_BASE`, which is the one input the whole
# seed-lottery argument above rests on.
#
# What the key provably covers: the request (op, planner config,
# `condition2_resolutions`, and the set's robot, group, config, seed,
# `motion_resolution`, objects and the problem itself), the planner seed, the
# fixture pair's contents, and the oracle sources plus its resolved non-file
# build inputs. What it does not: the wall clock, this machine's load, and the
# thread scheduling inside the oracle -- so a reused record's `wall_secs` is
# the producing run's, never this one's. `JOBS` is deliberately absent; it
# changes how long the answer takes and not what it is.
#
# On reuse the record's own `planner_rng_seed` stamp is checked against the
# seed this run would have passed, and a mismatch is a hard failure. Under
# content addressing that is a corruption check rather than the primary gate,
# and it is cheap enough to keep as one.
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"

# Same self-copy-and-re-exec this file's own caller does, at its own top --
# see measure-phase8-optimizer-properties.sh's copy of this comment for the
# three hazards (corruption, version split, late swap). Independent of that
# caller rather than snapshotted BY it: this file is `usage`-documented and
# runnable standalone, and it is a fresh `bash` reading this file from
# $REPO_ROOT either way, invoked by a person directly or as this run's own
# "helper not yet reached" -- so it has to protect itself regardless of who
# starts it, not rely on a caller that may not have.
#
# `run-oracle.sh` ($ORACLE below) is NOT snapshotted here either, for the
# same reason and the same named exposure as the other script: it derives its
# own $REPO_ROOT from its own `$BASH_SOURCE[0]` and would need this same
# treatment, in a third file, to be safe to copy.
#
# $REPO_ROOT is not redirected -- everything that reads or builds from the
# real tree (fixtures/, `$BIN`'s `cargo build` fallback, git commands) still
# has to name it, not this file's own private copy.
if [[ -z "${_PHASE8_CPPBASE_REPO_ROOT:-}" ]]; then
  SNAP_ROOT="$(mktemp -d)"
  trap 'rm -rf "$SNAP_ROOT"' EXIT
  SELF="$SNAP_ROOT/self"
  # This file's own name, not a literal, so a rename cannot silently desync
  # the copy source from what this process was actually invoked as.
  SELF_NAME="$(basename "${BASH_SOURCE[0]}")"
  mkdir -p "$SELF/tools/ci" "$SELF/tools/moveit-oracle" || exit 1
  cp "$REPO_ROOT/tools/ci/$SELF_NAME" "$SELF/tools/ci/" || exit 1
  cp "$REPO_ROOT/tools/ci/gate-lib.sh" "$SELF/tools/ci/" || exit 1
  cp "$REPO_ROOT/tools/moveit-oracle/src-digest.sh" "$SELF/tools/moveit-oracle/" || exit 1
  export _PHASE8_CPPBASE_REPO_ROOT="$REPO_ROOT"
  export _PHASE8_CPPBASE_SNAPDIR="$SNAP_ROOT"
  exec "$SELF/tools/ci/$SELF_NAME" "$@"
fi

# Second execution, from the private copy. $REPO_ROOT comes back from the
# environment, not from `$BASH_SOURCE[0]` here -- re-deriving it would name
# $SELF, the copy, instead of the tree fixtures/ and the oracle actually live
# under.
REPO_ROOT="$_PHASE8_CPPBASE_REPO_ROOT"
SNAP_ROOT="$_PHASE8_CPPBASE_SNAPDIR"
trap 'rm -rf "$SNAP_ROOT"' EXIT

. "$(dirname "${BASH_SOURCE[0]}")/gate-lib.sh"

require_caller_tree "$REPO_ROOT"

usage() {
  cat >&2 <<'USAGE'
usage: measure-phase8-cpp-baseline.sh <chomp|stomp> <config> <count> <set_seed> <out_dir>

  config    obstacle configuration name (floor_wall, cage, ...)
  count     number of problems
  set_seed  seed the problem set itself is generated at (900001 / 900002)
  out_dir   created if absent; results land in <out_dir>/<planner>.<config>.ndjson

environment:
  SET_FILE           a problem set already emitted by plan_benchmark_problem_set.
                     Used instead of generating one, so the C++ side and a port
                     harness can be handed the SAME BYTES rather than two
                     generator runs that are only argued to agree. `config`,
                     `count` and `set_seed` are still required and are checked
                     against the file's own fields -- a mismatch is a hard
                     failure, because a set file naming a different population
                     than the arguments is exactly the substitution this option
                     would otherwise make silent.
  ROBOT              which fixture to generate for (default panda). Ignored
                     when SET_FILE is given: the robot is read from the set
                     either way, see below.
  PLANNER_SEED_BASE  per-problem planner RNG seed base (default 700001, the
                     same base the port harnesses use)
  CONDITION2_RESOLUTIONS
                     comma-separated extra densification resolutions; each
                     produces one more condition-2 verdict over the SAME path
                     in `condition2_by_resolution`. Unset: field absent.
  CHOMP_MAX_ITERATIONS   default 50, `ChompParameters`'s own constructor value
  STOMP_CLOCK_BOUND      wall-clock ceiling in seconds (default 3600)
  CHOMP_CLOCK_BOUND      wall-clock ceiling in seconds (default 3600)
  JOBS               parallel oracle processes (default 12)
USAGE
  exit 2
}

[ $# -eq 5 ] || usage
PLANNER="$1"
CONFIG="$2"
COUNT="$3"
SET_SEED="$4"
OUT_DIR="$5"

case "$PLANNER" in
  chomp | stomp) ;;
  *) usage ;;
esac

PLANNER_SEED_BASE="${PLANNER_SEED_BASE:-700001}"
# The operating-point grid condition 2 is re-evaluated on. It reaches the
# oracle as the request field `condition2_resolutions`, which the port
# harnesses read from the same request, so both sides provably walk the same
# grid. Empty -> a `null` the oracle treats as an absent field.
CONDITION2_RESOLUTIONS="${CONDITION2_RESOLUTIONS:-}"
if [ -n "$CONDITION2_RESOLUTIONS" ]; then
  C2_GRID_JSON="[$CONDITION2_RESOLUTIONS]"
else
  C2_GRID_JSON=null
fi
CHOMP_MAX_ITERATIONS="${CHOMP_MAX_ITERATIONS:-50}"
CHOMP_CLOCK_BOUND="${CHOMP_CLOCK_BOUND:-3600}"
STOMP_CLOCK_BOUND="${STOMP_CLOCK_BOUND:-3600}"
JOBS="${JOBS:-12}"

# solve_one's oracle call (gate-lib.sh's oracle_call) has to outlive the
# clock bound baked into its own request -- CHOMP_CLOCK_BOUND or
# STOMP_CLOCK_BOUND above, embedded as `planning_time_limit`/
# `allowed_planning_time`. Killing it any sooner would misreport a solve
# that is about to hit its OWN deadline and report TIMED_OUT as an external
# hang instead. 300s margin is for the oracle to notice its deadline and
# flush a response, not for more work -- every teardown this repo has
# measured is sub-second (`ORACLE_SHUTDOWN_TIMEOUT`'s doc,
# tools/moveit-diff/src/lib.rs).
if [ "$PLANNER" = chomp ]; then
  SOLVE_TIMEOUT=$((CHOMP_CLOCK_BOUND + 300))
else
  SOLVE_TIMEOUT=$((STOMP_CLOCK_BOUND + 300))
fi

ROBOT="${ROBOT:-panda}"
SET_FILE="${SET_FILE:-}"
ORACLE="$REPO_ROOT/tools/moveit-oracle/run-oracle.sh"
BIN="$REPO_ROOT/target/release/examples/plan_benchmark_problem_set"

mkdir -p "$OUT_DIR"
SET_JSON="$OUT_DIR/$CONFIG.$COUNT.$SET_SEED.set.json"
# One store for every run into this directory, addressed by content rather
# than by a name assembled from the inputs someone remembered. See the header:
# the names this used to be built from could not distinguish two planner seed
# bases over one set, which is the §286.5 experiment.
RES_DIR="$OUT_DIR/res"
mkdir -p "$RES_DIR"

if [ ! -x "$BIN" ]; then
  echo "building examples/plan_benchmark_problem_set (release)..." >&2
  cargo build --release --example plan_benchmark_problem_set -p cspace-planners \
    --manifest-path "$REPO_ROOT/Cargo.toml" >&2
fi
# Copied for the same reason $GEN/$CHOMP/$STOMP are copied after the build in
# measure-phase8-optimizer-properties.sh: a concurrent `cargo build` replacing
# $REPO_ROOT/target between here and the call below would swap the binary out
# from under this run. Only reached (this copy, and the call it protects) when
# SET_FILE is unset -- the caller that always sets it hands over a $BIN this
# copy never has to answer for.
mkdir -p "$SNAP_ROOT/bin" || exit 1
cp "$BIN" "$SNAP_ROOT/bin/" || exit 1
BIN="$SNAP_ROOT/bin/$(basename "$BIN")"

# The problem set is generated once and reused for every problem's request, so
# every shard is provably looking at the same 500 problems as the port run and
# as the other planner. `SET_FILE` goes one better: the caller hands over a set
# it has already run the port against, so "the same problems" is byte identity
# rather than an argument about two generator runs agreeing.
if [ -n "$SET_FILE" ]; then
  [ -s "$SET_FILE" ] || { echo "FAIL SET_FILE=$SET_FILE is empty or absent" >&2; exit 1; }
  cp "$SET_FILE" "$SET_JSON"
elif [ ! -s "$SET_JSON" ]; then
  "$BIN" "$CONFIG" "$COUNT" "$SET_SEED" "$ROBOT" >"$SET_JSON" 2>"$OUT_DIR/$CONFIG.stats"
fi

# The set's own account of which population it is, against the arguments that
# name the output file and the seeds. Without this a SET_FILE from another
# config lands in `<planner>.<config>.ndjson` under this config's name, and the
# per-problem seeds -- `PLANNER_SEED_BASE + id` -- would be the only thing left
# saying which problems were actually solved.
for field in config:CONFIG seed:SET_SEED; do
  key="${field%%:*}"
  want="$(eval printf '%s' "\$${field##*:}")"
  have="$(jq -r --arg k "$key" '.[$k] | tostring' "$SET_JSON")"
  if [ "$have" != "$want" ]; then
    echo "FAIL the problem set says $key=$have, the arguments say $want" >&2
    exit 1
  fi
done
have_count="$(jq '.problems | length' "$SET_JSON")"
if [ "$have_count" != "$COUNT" ]; then
  echo "FAIL the problem set holds $have_count problems, the arguments say $COUNT" >&2
  exit 1
fi

# The robot is the set's, not this file's. See the header: a constant here and
# a `robot` field there are two sources of truth for one fact, and the oracle
# fails obscurely (unknown joint model group) rather than clearly when they
# disagree.
SET_ROBOT="$(jq -r '.robot // empty' "$SET_JSON")"
if [ -z "$SET_ROBOT" ]; then
  echo "FAIL the problem set names no robot, so no fixture pair can be chosen for it" >&2
  exit 1
fi
URDF="$REPO_ROOT/fixtures/$SET_ROBOT.urdf"
SRDF="$REPO_ROOT/fixtures/$SET_ROBOT.srdf"
for f in "$URDF" "$SRDF"; do
  [ -s "$f" ] || { echo "FAIL the set names robot '$SET_ROBOT' but $f is absent" >&2; exit 1; }
done

# The three inputs a request's own bytes cannot carry: the two fixture files
# named in the oracle's argv, and the oracle itself. `oracle_stamp` is the
# same digest run-oracle.sh insists the image carries, so it names the binary
# that will actually answer rather than a tag that could have moved. The
# function comes from the private copy, the directory it hashes stays
# $REPO_ROOT -- see the same split where this file's own bootstrap sources
# gate-lib.sh, above.
# shellcheck source=tools/moveit-oracle/src-digest.sh
source "$(dirname "${BASH_SOURCE[0]}")/../moveit-oracle/src-digest.sh"
ORACLE_STAMP="$(oracle_stamp "$REPO_ROOT/tools/moveit-oracle")"
URDF_SHA="$(sha256sum <"$URDF" | cut -d' ' -f1)"
SRDF_SHA="$(sha256sum <"$SRDF" | cut -d' ' -f1)"

# Requests are regenerated from the set on every run, so they are not a cache
# and need no key -- a fresh directory per run means two concurrent runs into
# one OUT_DIR cannot overwrite each other's stdin. Kept on failure, because
# the request is the first thing to read when a problem does not solve.
REQ_DIR="$(mktemp -d "$OUT_DIR/req.XXXXXXXX")"
cleanup_requests() {
  local rc=$?
  # $SNAP_ROOT (the self-copy made at the top of this file) is unconditional,
  # unlike $REQ_DIR below: nothing about a failure makes the copy worth
  # keeping to read.
  rm -rf "$SNAP_ROOT"
  if [ "$rc" -eq 0 ]; then
    rm -rf "$REQ_DIR"
  else
    echo "requests kept at $REQ_DIR" >&2
  fi
}
trap cleanup_requests EXIT

# One single-problem request per problem, carrying the whole object list and
# the set's own `motion_resolution` unchanged -- only `problems` is narrowed.
# `.op` is rewritten rather than regenerated so the geometry and the endpoints
# are literally the same bytes the `plan` op and the port harnesses consume.
if [ "$PLANNER" = chomp ]; then
  PLANNER_CFG=$(printf '{"max_iterations":%s,"planning_time_limit":%s}' \
    "$CHOMP_MAX_ITERATIONS" "$CHOMP_CLOCK_BOUND")
  jq -c --argjson cfg "$PLANNER_CFG" --argjson c2 "$C2_GRID_JSON" \
    '.op="chomp_plan" | .chomp=$cfg | .condition2_resolutions=$c2 | . as $r
     | range(0; $r.problems|length) as $i
     | ($r | .problems=[$r.problems[$i]])' "$SET_JSON" \
    | while IFS= read -r line; do
        id=$(printf '%s' "$line" | jq -r '.problems[0].id')
        printf '%s\n' "$line" >"$REQ_DIR/$id.json"
      done
else
  PLANNER_CFG=$(printf '{"allowed_planning_time":%s}' "$STOMP_CLOCK_BOUND")
  jq -c --argjson cfg "$PLANNER_CFG" --argjson c2 "$C2_GRID_JSON" \
    '.op="stomp_plan" | .stomp=$cfg | .condition2_resolutions=$c2 | . as $r
     | range(0; $r.problems|length) as $i
     | ($r | .problems=[$r.problems[$i]])' "$SET_JSON" \
    | while IFS= read -r line; do
        id=$(printf '%s' "$line" | jq -r '.problems[0].id')
        printf '%s\n' "$line" >"$REQ_DIR/$id.json"
      done
fi

# The whole input to one oracle process, hashed: its stdin, its seed, the two
# fixture files it is handed, and the sources the answering binary was built
# from. Nothing here enumerates the options that shape a request -- they are
# already in the bytes -- which is what makes forgetting one unrepresentable
# rather than merely noticeable.
cache_key() {
  local id="$1"
  {
    printf 'oracle-src %s\nurdf %s\nsrdf %s\nplanner-rng-seed %s\nrequest\n' \
      "$ORACLE_STAMP" "$URDF_SHA" "$SRDF_SHA" "$((PLANNER_SEED_BASE + id))"
    cat "$REQ_DIR/$id.json"
  } | sha256sum | cut -d' ' -f1
}

solve_one() {
  local id="$1"
  local seed=$((PLANNER_SEED_BASE + id))
  local key
  key="$(cache_key "$id")"
  local out="$RES_DIR/$key.json"
  if [ -s "$out" ]; then
    # The stamp `:249` writes, read back. Content addressing already makes a
    # seed mismatch here impossible without corruption, and that is exactly
    # why it is worth failing on: a record whose own account of its seed
    # disagrees with its key is not a result, whatever else it is.
    local had
    had="$(jq -r '.planner_rng_seed // "absent"' "$out")"
    if [ "$had" != "$seed" ]; then
      echo "FAIL cached $out records planner_rng_seed=$had, this run needs $seed" >&2
      return 1
    fi
    return 0
  fi
  local started
  started=$(date +%s.%N)
  # No pipe into jq here: a pipeline's status is its last stage's, so an oracle
  # that died would be recorded as a well-formed empty result.
  if ! oracle_call "$SOLVE_TIMEOUT" -- \
       sg docker -c "$ORACLE --urdf $URDF --srdf $SRDF --planner-rng-seed $seed" \
       <"$REQ_DIR/$id.json" >"$out.raw" 2>"$out.err"; then
    oracle_call_explain "$ORACLE_CALL_STATUS" "problem $id: "
    echo "  see $out.err" >&2
    return 1
  fi
  local elapsed
  elapsed=$(echo "$(date +%s.%N) - $started" | bc)
  # Written aside and renamed, so a run killed mid-`jq` leaves no truncated
  # record for the next one to accept as a complete answer.
  jq -c --argjson secs "$elapsed" --argjson seed "$seed" \
    '.result.problems[0] + {planner_rng_seed: $seed, wall_secs: $secs}' \
    <"$out.raw" >"$out.part"
  mv "$out.part" "$out"
  rm -f "$out.raw"
}
export -f cache_key solve_one oracle_call oracle_call_explain
export RES_DIR REQ_DIR ORACLE URDF SRDF PLANNER_SEED_BASE ORACLE_STAMP URDF_SHA SRDF_SHA SOLVE_TIMEOUT

echo "=== $PLANNER / $SET_ROBOT / $CONFIG: $COUNT problems, $JOBS jobs, seed base $PLANNER_SEED_BASE ===" >&2
seq 0 $((COUNT - 1)) | xargs -P "$JOBS" -I{} bash -c 'solve_one {}'

OUT="$OUT_DIR/$PLANNER.$CONFIG.ndjson"
: >"$OUT"
for id in $(seq 0 $((COUNT - 1))); do
  key="$(cache_key "$id")"
  if [ ! -s "$RES_DIR/$key.json" ]; then
    echo "FAIL problem $id produced no result" >&2
    exit 1
  fi
  cat "$RES_DIR/$key.json" >>"$OUT"
done

# The clock-bound check the header describes: any TIMED_OUT means the wall
# clock, not the iteration bound, decided that problem, and the whole rate is
# then partly a measurement of this machine.
timed_out=$(jq -s '[.[] | select(.failure == "TIMED_OUT")] | length' "$OUT")
solved=$(jq -s '[.[] | select(.solved)] | length' "$OUT")
echo "$PLANNER/$SET_ROBOT/$CONFIG: solved $solved/$COUNT, timed_out $timed_out -> $OUT" >&2
if [ "$timed_out" -ne 0 ]; then
  echo "FAIL $timed_out problems hit the wall-clock bound; this rate measures the machine" >&2
  exit 1
fi
