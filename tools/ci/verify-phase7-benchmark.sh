#!/bin/bash
# Phase 7's three completion conditions (PORTING-PLAN.md §5), as a command
# rather than as numbers in a report.
#
#   1. success rate over 500 benchmark problems >= 90% of C++ OMPL RRTConnect's
#   2. 100% of produced paths pass cspace-scene's collision check and constraints
#   3. median path length within 1.3x of C++ OMPL's
#
# Conditions 1 and 3 need the C++ baseline, so they need docker and the
# `moveit-rs/oracle` image -- the same reason `verify-oracle-sweep.sh` is a
# `verify-*.sh` and not a `check-*.sh` or a `cargo test`. Condition 2 needs
# neither and is measured on every set, including the constrained one the
# oracle's `plan` op cannot express (see
# `examples/plan_benchmark_problem_set.rs`'s `# Path constraints`).
#
# # Why this is opt-in even among the verify-*.sh set
#
# `verify-all.sh` runs every `tools/ci/verify-*.sh` by glob, per merge round.
# The full Phase 7 gate is ~40 minutes of wall clock (measured; see
# `doc/phase7-benchmark-results.json`), which is an order of magnitude more
# than any other member of that set and far more than a per-round cost should
# absorb. So the default run here is a *pilot*: the same harness, same seeds,
# same code path, at PILOT_COUNT problems per config instead of 250, plus
# both condition-2 injection gates in full. That keeps the script honest as a
# glob member -- it really does run, and it really would catch a broken
# harness or a regressed injection gate -- while the load-bearing 500-problem
# gate is requested explicitly:
#
#   tools/ci/verify-phase7-benchmark.sh full
#
# The pilot prints a loud note that its numbers are NOT the Phase 7 gate.
# A small run reported as though it were the gate is the failure this
# split exists to prevent.
#
# # Reproducing
#
#   sg docker -c 'tools/ci/verify-phase7-benchmark.sh full'
#
# Every seed is fixed and printed. `full` writes the complete result set to
# doc/phase7-benchmark-results.json.
set -uo pipefail

MODE="${1:-pilot}"
case "$MODE" in
  pilot|full) ;;
  *) echo "usage: $0 [pilot|full]" >&2; exit 2 ;;
esac

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"

. "$REPO_ROOT/tools/ci/gate-lib.sh"

require_caller_tree "$REPO_ROOT"
cd "$REPO_ROOT" || exit 1  # no `set -e` here: a failed cd would
                           # otherwise run this gate in the caller's tree

ORACLE="$REPO_ROOT/tools/moveit-oracle/run-oracle.sh"
GEN="$REPO_ROOT/target/release/examples/plan_benchmark_problem_set"
PORT="$REPO_ROOT/target/release/examples/plan_benchmark_port"
RESULTS="$REPO_ROOT/doc/phase7-benchmark-results.json"

# Per-problem planner timeout, seconds. A call that hits it is a FAILURE,
# never a skip -- see `plan_benchmark_port.rs`'s `DEFAULT_TIMEOUT_SECONDS`
# for how this number was sized and why erring high is the safe direction.
TIMEOUT_SECONDS=120

# Bounds for this script's own oracle round trips (gate-lib.sh's
# `oracle_call`), one per call SHAPE rather than one constant for all three:
# a bound generous enough for the escalation call below would be uselessly
# loose on the cheap per-path checks, and one tuned to those would kill a
# legitimate escalated retry early.
#
#   ORACLE_PATH_CHECK_TIMEOUT   oracle_path_check's is_state_valid call: no
#     search, no request-level clock bound of its own. Directly measured
#     below at "~9s" for the injection gate's small batches; the condition-2
#     cross-check hands the same function every solved path from a whole
#     config (up to 250), so this is sized well above either, not just the
#     measured case.
#   ORACLE_BASELINE_TIMEOUT   the C++ OMPL RRTConnect baseline over a whole
#     config's problem set: iteration-capped per problem
#     (plan_benchmark_problem_set.rs's `max_iterations: 2000`) but no
#     wall-clock cap of its own. No exact wall clock for this call alone has
#     been pinned (only the whole script's ~40 minutes, see the header); far
#     above CHOMP's comparable 500-problem/170s run, far below the
#     escalation bound below.
#   ORACLE_ESCALATION_TIMEOUT   the feasibility-escalation call: up to 50000
#     iterations (full mode) with no wall-clock cap in the request at all,
#     retrying exactly the problems that already failed to converge once --
#     the header below documents these as routinely running to their full
#     iteration budget, and a comment there also names 600s in prose for
#     that observation, not the port-side `ESCALATED_TIMEOUT` this bounds
#     differently from (that governs `run_port_sharded`, a separate call).
#     3600s matches this codebase's own established default for "a real but
#     self-terminating long C++ solve" (`CHOMP_CLOCK_BOUND`/
#     `STOMP_CLOCK_BOUND`, both default 3600 in
#     measure-phase8-cpp-baseline.sh) -- this is the least-measured of the
#     three and the one most worth revisiting once an actual wall clock for
#     it is recorded.
ORACLE_PATH_CHECK_TIMEOUT="${ORACLE_PATH_CHECK_TIMEOUT:-600}"
ORACLE_BASELINE_TIMEOUT="${ORACLE_BASELINE_TIMEOUT:-1800}"
ORACLE_ESCALATION_TIMEOUT="${ORACLE_ESCALATION_TIMEOUT:-3600}"

# 250 per config x 2 configs = the 500 problems §5 names. The pilot count is
# small enough to stay inside a per-round budget and large enough that a
# harness that silently produces nothing is still caught.
FULL_COUNT=250
# Overridable only because it cannot be used to fake coverage: the pinned
# `problems` count is checked (`pin-population` in the decision list below),
# so a run at a reduced count fails rather than reporting a smaller set as a
# pass. It exists so a mutation proof -- break a check, show it fails, restore
# -- costs seconds instead of the 5m12s a full pilot takes.
PILOT_COUNT="${PILOT_COUNT:-8}"

if [[ "$MODE" == "full" ]]; then
  COUNT="$FULL_COUNT"
else
  COUNT="$PILOT_COUNT"
fi

# robot config count seed -- the seed is the reproducibility contract, so it
# lives here, in the committed harness, rather than in a round brief. panda's
# two seeds are the ones `benches/sweep_baseline.sh` already measured its C++
# baseline at; changing them would silently invalidate that comparison.
#
# That last sentence is a claim about the *generator*, not just the seeds: the
# round that added fanuc rewrote `plan_benchmark_problem_set.rs` (robot table,
# reach-scaled geometry, constraint filter), and if that rewrite moved panda's
# problems then the older C++ baseline is a measurement of a different set.
# Here is the command that checks it, so the claim is not left as an assertion
# -- `5adbed6` is the last commit before that rewrite:
#
#   OLD=crates/cspace-planners-sbp/examples/plan_benchmark_problem_set_pre_5adbed6.rs
#   git show 5adbed6:crates/cspace-planners-sbp/examples/plan_benchmark_problem_set.rs >"$OLD"
#   cargo build --release -p cspace-planners-sbp \
#     --example plan_benchmark_problem_set_pre_5adbed6 --example plan_benchmark_problem_set
#   for s in "floor_wall 250 900001" "cage 250 900002"; do
#     diff <(./target/release/examples/plan_benchmark_problem_set_pre_5adbed6 $s 2>/dev/null) \
#          <(./target/release/examples/plan_benchmark_problem_set $s panda 2>/dev/null \
#            | jq -c 'del(.robot,.config,.scale,.joint_constraint)') \
#       && echo "IDENTICAL $s"
#   done
#   rm -f "$OLD"
#
# The `del(...)` is not a thumb on the scale, and running the diff without it
# is worth doing first: the new generator emits four *additional* keys
# (`robot`, `config`, `scale`, `joint_constraint`) and removes none, so the raw
# diff differs on exactly those and says nothing about the problems. Measured
# 2026-08-06 on both seeds: added `config,joint_constraint,robot,scale`,
# removed nothing, and every remaining key -- `problems`, `objects`, `range`,
# `motion_resolution`, `max_iterations`, `seed`, `group`, `op`, `id` -- byte
# identical. That the four additions also leave the *oracle* unmoved is not
# argued from nlohmann's unknown-key behaviour but measured: fed the new
# requests it returns 250/250 and 248/250 exact with pooled median
# 2.6597767032746464, the same figures §131 recorded from the old ones.
# §219.3 records the run.
SETS=(
  "panda floor_wall $COUNT 900001"
  "panda cage       $COUNT 900002"
  "fanuc floor_wall $COUNT 900021"
  "fanuc cage       $COUNT 900022"
)

# Condition 2's constrained population. Port-side only: the oracle's `plan`
# op has no constraint input, so this set has no C++ counterpart and takes no
# part in conditions 1 and 3.
CONSTRAINED_SET="panda floor_wall $COUNT 900011 panda_joint1:0.0:0.5"

# `seed_base` for the port planner's own RNG, independent of the request's
# OMPL seed. §131.2 recorded the absence of this number as a defect: the
# reproduction procedure was documented while the input it needs was not, so
# a later reproduction had to invent one and got different numbers. It is
# committed here for that reason.
SEED_BASE=424242

# Regression pins -- separate from §5's three thresholds, and needed because
# those thresholds have room for a large real regression to pass:
#
#   condition 1 for panda demands port >= 0.9 x 498/500, i.e. 449 of 500
#   problems. The port solves 497. A port that lost 48 problems still passes.
#   condition 3 allows 1.3x the C++ median; the measured ratio is 1.023, so a
#   port whose paths got 27% longer still passes.
#
# So the pins are floors under the measured values, and they are sized from
# the measured stream-to-stream spread rather than picked: a pin tighter than
# the noise fails on an innocent change, a pin looser than a regression sees
# nothing. `problems` is part of the pin so a value measured at n=500 can
# never be compared against a run of a different size.
#
# panda, port side, 500 problems (`floor_wall` 250 + `cage` 250), seed_base
# 424242 / 1 / 20260806 / 999983 -- only the port RNG stream changes:
#   solved 497 / 498 / 497 / 497,  pooled median 2.6576 .. 2.7154
# panda, C++ side, same 500 problems, request seed 900001+900002 (the pinned
# pair) / 1 / 20260806 / 999983 -- only the OMPL stream changes:
#   exact 498 / 497 / 498 / 496,  pooled median 2.6544 .. 2.6684
# fanuc, port side, 500 problems, seed_base 424242 / 20260806 / 999983:
#   solved 405 / 405 / 405,       pooled median 1.8310 .. 1.8929
# fanuc, C++ side, seed 900021+900022 (pinned) / 1 / 20260806 / 999983:
#   exact 406 / 405 / 405 / 405,  pooled median 1.8369 .. 1.8734
#
# Worst ratio over those streams: panda 0.99596 .. 1.02296, fanuc 0.97739 ..
# 1.03044. The ceilings are 1.10 -- above the whole spread, and 3x tighter
# than §5's 1.30. The floors are 0.90: condition 3 is one-sided, so a port
# whose median collapsed (paths that no longer go around what they should)
# passes it by getting better-looking, and 0.90 is ~4x the observed shortfall.
# The solved floors sit 5 problems (1% of the set) under the lowest stream,
# i.e. 5x the largest observed stream difference of 1 problem.
#
# `cpp_solved_floor` is here because condition 1's bar IS the C++ rate: a
# degraded baseline (a scene that failed to load its objects, an OMPL that
# stopped searching) lowers the bar instead of failing the run.
#
# pilot pins are separate values measured from a pilot run, not the full-mode
# ones scaled: at 8 problems per config a floor derived from 500 says nothing.
# Measured pilot run (`PILOT_COUNT=8`, 16 problems per stratum, wall clock
# 5m12s): panda port 16/16 and C++ 16/16 at pooled ratio 0.997x, fanuc port
# 12/12 and C++ 12/12 at 1.004x. The floors are one problem under each, which
# is as tight as a 16-problem set allows -- at this size one problem is 6% of
# the set, so a pilot pin is a plumbing tripwire and not a rate measurement.
PINS_ALL='{
  "full": {
    "panda": {"problems": 500, "port_solved_floor": 492, "cpp_solved_floor": 491,
              "ratio_ceiling": 1.10, "ratio_floor": 0.90},
    "fanuc": {"problems": 500, "port_solved_floor": 400, "cpp_solved_floor": 400,
              "ratio_ceiling": 1.10, "ratio_floor": 0.90}
  },
  "pilot": {
    "panda": {"problems": 16, "port_solved_floor": 15, "cpp_solved_floor": 15,
              "ratio_ceiling": 1.10, "ratio_floor": 0.90},
    "fanuc": {"problems": 16, "port_solved_floor": 11, "cpp_solved_floor": 11,
              "ratio_ceiling": 1.10, "ratio_floor": 0.90}
  }
}'
PINS_JSON="$(jq -c --arg m "$MODE" '.[$m] // {}' <<<"$PINS_ALL")"

# How many port processes one set is split across. Sharding is exact, not an
# approximation: each problem gets its own `PlanningScene` and its own RNG
# seeded `SEED_BASE + problem.id`, so which process runs a given problem
# changes nothing about its result -- only the wall clock. Without this the
# escalation stage alone runs for hours, because every problem it retries is
# one that already failed once and therefore runs to its full deadline.
#
# Capped well under `nproc` so a machine shared with other work (this repo's
# caucus panels run concurrently) is not oversubscribed; the port planner is
# single-threaded, so each shard is one core.
SHARDS="${SHARDS:-16}"

WORKDIR="$(mktemp -d)"
trap 'rm -rf "$WORKDIR"' EXIT

# Runs the port planner over $1's problems, split across $SHARDS processes,
# concatenating the per-problem NDJSON lines into $2. The per-shard summary
# lines are dropped: every aggregate this script reports is recomputed from
# the per-problem lines instead (see `port_aggregate`), which is both what
# makes sharding transparent and an independent check on the binary's own
# summary arithmetic.
run_port_sharded() {
  local request="$1" out="$2" timeout="$3" dense="${4:-}"
  local pids=() i rc=0
  # A fresh directory per call, named after the output file. Fixed shard
  # filenames in one shared $WORKDIR were a real defect the pilot caught: a
  # later call whose problem set is smaller writes fewer shard files, and the
  # concatenation loop then picked up the *previous* call's leftovers, so an
  # escalation over 2 problems reported 6 solved and the unknown count went
  # negative. Per-call isolation makes that unrepresentable rather than
  # something to remember to clean up.
  local dir
  dir="$WORKDIR/shards.$(basename "$out")"
  rm -rf "$dir"
  mkdir -p "$dir"
  : >"$out"
  for ((i = 0; i < SHARDS; i++)); do
    jq --argjson k "$SHARDS" --argjson i "$i" \
      '.problems = [.problems[] | select((.id % $k) == $i)]' \
      "$request" >"$dir/shard.$i.json"
    # Skip an empty shard so a small set does not spawn $SHARDS processes to
    # do nothing.
    if [[ "$(jq '.problems|length' "$dir/shard.$i.json")" == "0" ]]; then
      continue
    fi
    "$PORT" "$SEED_BASE" "$timeout" "" "$dense" \
      <"$dir/shard.$i.json" >"$dir/shard.$i.ndjson" 2>"$dir/shard.$i.err" &
    pids+=("$!")
  done
  for pid in "${pids[@]}"; do wait "$pid" || rc=1; done
  for ((i = 0; i < SHARDS; i++)); do
    [[ -s "$dir/shard.$i.ndjson" ]] || continue
    jq -c 'select(.id != null)' "$dir/shard.$i.ndjson" >>"$out"
  done

  # Every problem in, every problem out. A shard that died mid-run would
  # otherwise silently shrink the denominator, which reads as a better
  # success rate rather than as a lost shard.
  local want have
  want="$(jq '.problems|length' "$request")"
  have="$(wc -l <"$out")"
  if [[ "$want" != "$have" ]]; then
    echo "FAIL sharded port run lost problems: requested $want, got $have ($out)" >&2
    for ((i = 0; i < SHARDS; i++)); do
      [[ -s "$dir/shard.$i.err" ]] && tail -3 "$dir/shard.$i.err" >&2
    done
    rc=1
  fi
  return $rc
}

# Every port-side aggregate Phase 7 needs, recomputed from per-problem lines.
# `timeout` is counted as a failure of its own kind, never folded into
# `solved` and never dropped.
port_aggregate() {
  jq -s '
    def median: sort | if length==0 then null
      elif (length%2)==1 then .[length/2|floor]
      else (.[length/2-1]+.[length/2])/2 end;
    # Per-problem lines only. `run_port_sharded` already drops the binary`s
    # own summary line, but the unsharded injection runs still carry one, and
    # counting it would make `problems` one too many everywhere.
    [.[] | select(.id != null)] |
    {
      problems: length,
      solved: ([.[]|select(.outcome=="solved")]|length),
      timeouts: ([.[]|select(.outcome=="timeout")]|length),
      failures: ([.[]|select(.outcome!="solved" and .outcome!="timeout")]|length),
      median_length: ([.[]|select(.outcome=="solved")|.length]|median),
      condition2_checked: ([.[]|select(.condition2_valid!=null)]|length),
      condition2_pass: ([.[]|select(.condition2_valid==true)]|length),
      waypoints_checked: ([.[]|select(.waypoints_checked!=null)|.waypoints_checked]|add // 0),
      # What `densify` was handed, against what it produced. A densification
      # that collapsed back to the vertices the planner itself produced would
      # re-check only states `PlanningSceneValidityChecker` already accepted
      # during the search -- exactly the independent-path cross-check that
      # condition 2 is supposed to be -- while reporting 100%.
      raw_waypoints: ([.[]|select(.raw_waypoints!=null)|.raw_waypoints]|add // 0),
      # Largest distance from the first waypoint of a returned path to the
      # requested start, or from its last to the requested goal, over every
      # solved problem. `outcome: "solved"` alone does not say WHICH problem
      # was solved; see `# Endpoint fidelity` in `plan_benchmark_port.rs`.
      max_endpoint_gap: ([.[]|select(.outcome=="solved")|(.start_gap//0),(.goal_gap//0)]|max // 0),
      cpu_seconds: ([.[]|.plan_seconds]|add // 0),
      slowest_seconds: ([.[]|.plan_seconds]|max),
      slowest_problem_id: (max_by(.plan_seconds)|.id)
    }' "$1"
}

# Snapshotted HERE, before the build, and not where it is written out at the
# end. The digest has to describe the source this run compiled; taken after the
# measurement it describes whatever is on disk hours later instead, and an edit
# landing mid-run -- a parallel worker, a fix to the very harness whose numbers
# are being produced -- would be recorded as the code that ran. That record
# then reads as current precisely when it is furthest from true.
#
# `|| exit 1` on the digest rather than inlining it into `printf`: a failed
# digest must abort the run, not land in the record as an empty value that a
# later reader would see as drift.
# Same reason, same moment: `working_tree_dirty` and `dirty_paths` describe the
# tree this run measured, and taken at write time they would describe the tree
# hours of parallel work later. They also carry what the digests structurally
# cannot -- `git status --porcelain` lists untracked files, which are invisible
# to the index-scoped `git ls-files` inside `measured_source_digest` -- so the
# two fields only compose if they are captured together.
DIRTY_LIST="$(cd "$REPO_ROOT" && git status --porcelain)"
if [[ -n "$DIRTY_LIST" ]]; then TREE_DIRTY=true; else TREE_DIRTY=false; fi

# Same snapshot moment, for a fact the source digest cannot see. `oracle_stamp`
# (tools/moveit-oracle/src-digest.sh) hashes the oracle's files together with
# its resolved, env-overridable ORACLE_BASE_IMAGE/ORACLE_MOVEIT2_PACKAGES/
# ORACLE_MOVEIT2_SHA build inputs. `measured_source_digest("tools/moveit-oracle/src")`
# below is index-scoped to `src/`; the Dockerfile, build.sh, entrypoint.sh and
# the pinned MOVEIT2_SHA all live outside that directory, so repinning the
# oracle's moveit2 checkout changes which C++ numbers this run produces
# without moving that digest at all. run-oracle.sh checks the running image's
# stamp against this same value on every call, but that only catches a STALE
# image -- it never writes what it checked into anything the artifact keeps,
# so two runs made under two different pins each pass their own check and
# leave artifacts whose measured_sources cannot tell them apart. The image
# TAG is not recorded separately: `oracle_image_tag` (src-digest.sh) derives
# it from the stamp by truncating to the first 16 hex characters, so the tag
# is recoverable from the stamp and not the reverse -- recording the stamp is
# strictly more informative, and `oracle_image_tag "$ORACLE_STAMP"` recovers
# the tag whenever something needs to `docker image inspect` it.
# shellcheck source=tools/moveit-oracle/src-digest.sh
source "$REPO_ROOT/tools/moveit-oracle/src-digest.sh"
ORACLE_STAMP="$(oracle_stamp "$REPO_ROOT/tools/moveit-oracle")" || exit 1

# The list below is the SBP arm's own algorithm/dispatch crates plus the
# validity/cost code every arm calls to decide what it reports -- see the
# closure argument on `measured_source_digest` in gate-lib.sh for how this
# list was derived and what was deliberately left out.
if ! SOURCES_JSON="$(cd "$REPO_ROOT" && for f in \
    tools/ci/verify-phase7-benchmark.sh \
    crates/cspace-planners-sbp/examples/plan_benchmark_problem_set.rs \
    crates/cspace-planners-sbp/examples/plan_benchmark_port.rs \
    crates/cspace-planners-sbp/src \
    crates/cspace-planning/src \
    crates/cspace-planning/src/planner_registry \
    crates/cspace-collision/src \
    crates/cspace-planning/src/scene \
    crates/cspace-planning/src/constraints \
    tools/moveit-oracle/src; do
  d="$(measured_source_digest "$f")" || exit 1
  printf '%s %s\n' "$f" "$d"
done | jq -R -s 'split("\n")|map(select(length>0)|split(" "))|map({key:.[0],value:.[1]})|from_entries')"; then
  echo "FAIL could not digest this run's measured sources -- refusing to start a" >&2
  echo "  measurement whose record could not say what code produced it" >&2
  exit 1
fi

echo "=== building benchmark binaries (release) ==="
cargo build --release --manifest-path "$REPO_ROOT/Cargo.toml" \
  -p cspace-planners-sbp --examples || exit 1

failed=()

# --- condition 2's independent cross-check ---------------------------------
#
# `is_path_valid` (what condition 2 reports) and `DiscreteMotionValidator`
# (what the planner used while searching) are two entry points to one
# `ParryCollisionEnv`, and `build_injected_state` finds its bad state by
# asking that same env through a third. So the injection gate below proves
# the entry points agree with each other -- not that the collision model is
# right. A backend that misses a contact produces a colliding path AND
# approves it, and every check that only ever asks this port stays green.
# Only a different implementation can see that.
#
# The oracle's `is_state_valid` op is one: upstream MoveIt's
# `PlanningScene::isPathValid` over FCL. Its loop is per-waypoint with no
# interpolation of its own (`moveit_core/planning_scene/src/planning_scene.cpp
# :2365-2424`, read at the pinned sha), the same shape as this port's
# `is_path_valid`, so handing it the *same* densified waypoint list the port
# just checked asks two implementations exactly the same question about
# exactly the same states -- where a re-plan on the C++ side would be a
# different path and could only ever be compared statistically. That per-
# waypoint shape is also why several paths can share one oracle invocation:
# no waypoint's verdict depends on its neighbours, and `goal_constraints` is
# left empty so `isPathValid`'s own last-waypoint goal check never fires.
#
# $1 label, $2 robot, $3 the request JSON the paths came from (for `objects`),
# $4 port NDJSON carrying `dense`, $5 expected verdict (`valid`|`invalid`),
# $6 joint-constraint spec or "".
oracle_path_check() {
  local label="$1" robot="$2" request="$3" ndjson="$4" expect="$5" spec="$6"
  local isv="$WORKDIR/isv.$label.ndjson" out="$WORKDIR/isv.$label.out"
  local pc='{}'
  if [[ -n "$spec" ]]; then
    # `<joint>:<position>:<tolerance>` -- the same string
    # `parse_joint_constraint` rebuilds the port-side set from, in
    # `moveit_msgs::Constraints` shape. Symmetric tolerance and weight 1.0
    # because that is what `JointConstraint::new` was handed there.
    pc="$(jq -c -n --arg spec "$spec" '($spec|split(":")) as $p |
      {joint_constraints:[{joint_name:$p[0], position:($p[1]|tonumber),
        tolerance_above:($p[2]|tonumber), tolerance_below:($p[2]|tonumber),
        weight:1.0}]}')"
  fi
  jq -c --slurpfile req "$request" --argjson pc "$pc" \
    'select(.dense != null)
     | {id, op:"is_state_valid", objects: $req[0].objects,
        path_constraints: $pc, waypoints: .dense}' "$ndjson" >"$isv"

  # An empty comparison is the failure this whole cross-check exists to
  # prevent, so it is named rather than reported as agreement.
  local n_paths
  n_paths="$(wc -l <"$isv")"
  if [[ "$n_paths" -eq 0 ]]; then
    echo "  FAIL cross-check $label: no port path carried \`dense\`, nothing was compared" >&2
    failed+=("cross-check $label (nothing compared)")
    return 1
  fi
  if ! oracle_call "$ORACLE_PATH_CHECK_TIMEOUT" -- \
       sg docker -c "$ORACLE --urdf $REPO_ROOT/fixtures/$robot.urdf --srdf $REPO_ROOT/fixtures/$robot.srdf" \
       <"$isv" >"$out" 2>"$WORKDIR/isv.$label.err"; then
    oracle_call_explain "$ORACLE_CALL_STATUS" "  cross-check $label: "
    tail -5 "$WORKDIR/isv.$label.err" >&2
    failed+=("cross-check $label (oracle run)")
    return 1
  fi

  local report
  report="$(jq -n --slurpfile o <(jq -s '.' "$out") \
                  --slurpfile p <(jq -s 'map(select(.dense!=null))' "$ndjson") \
                  --arg expect "$expect" '
    ($p[0] | map({key:(.id|tostring), value:.}) | from_entries) as $by |
    [ $o[0][] | ($by[.id|tostring]) as $pp |
      { id,
        ok: (.ok == true),
        oracle_valid: .result.valid,
        port_valid: $pp.condition2_valid,
        # Index sets, not counts: two checkers that reject the same *number*
        # of different waypoints have not given the same answer.
        same_indices: ((.result.invalid_waypoints // []) == ($pp.invalid_waypoints // [])),
        n_oracle: ((.result.invalid_waypoints // [])|length),
        n_port: (($pp.invalid_waypoints // [])|length) } ] as $rows |
    { paths: ($rows|length),
      not_ok: [$rows[]|select(.ok|not)|.id],
      wrong_verdict: [$rows[]|select(.oracle_valid != ($expect == "valid"))|.id],
      disagree: [$rows[]|select(.oracle_valid != .port_valid)|.id],
      index_mismatch: [$rows[]|select(.same_indices|not)|{id, n_oracle, n_port}],
      waypoints: ([$o[0][]|.result.invalid_waypoints]|length) }')"
  local paths not_ok wrong disagree mismatch
  paths="$(jq -r '.paths' <<<"$report")"
  not_ok="$(jq -r '.not_ok|length' <<<"$report")"
  wrong="$(jq -r '.wrong_verdict|length' <<<"$report")"
  disagree="$(jq -r '.disagree|length' <<<"$report")"
  mismatch="$(jq -r '.index_mismatch|length' <<<"$report")"
  if [[ "$not_ok" == "0" && "$wrong" == "0" && "$disagree" == "0" && "$mismatch" == "0" ]]; then
    echo "  PASS cross-check $label: upstream isPathValid agrees on all $paths path(s), expected=$expect, invalid-index sets identical"
    return 0
  fi
  echo "  FAIL cross-check $label: paths=$paths not_ok=$not_ok wrong_verdict=$wrong port/oracle_disagree=$disagree index_mismatch=$mismatch" >&2
  jq -c '{not_ok, wrong_verdict, disagree, index_mismatch}' <<<"$report" >&2
  failed+=("cross-check $label")
  return 1
}

# --- condition 2's discrimination gate -------------------------------------
#
# Runs FIRST, and a failure here invalidates every condition-2 number below.
# A validator that silently checks nothing reports 100% exactly as a working
# one does; these runs splice a state verified bad by direct query into every
# solved path and require the checker to reject all of them. See
# `plan_benchmark_port.rs`'s `# Proving the condition-2 check can fail`.
#
# Run on EVERY robot/config in SETS, not only panda's `floor_wall`: the state
# `build_injected_state` splices is found by sampling *this* scene, the
# geometry differs per config, and fanuc's links and reach-scaled obstacles
# are a different collision problem entirely. A checker permissive for one
# scene's geometry passed the single-scene version of this gate.
#
# `INJECT_COUNT` is small because the property is per-path ("every solved path
# is rejected"), not statistical: 4 paths per scene across 4 scenes says
# strictly more than 6 paths in one scene, and costs less on fanuc, where a
# single call averages ~9 s.
INJECT_COUNT=4
echo
echo "=== condition 2 discrimination gate (injected bad waypoints must be rejected) ==="
for entry in "${SETS[@]}"; do
  read -r robot config count seed <<<"$entry"
  tag="${robot}_${config}"
  "$GEN" "$config" "$INJECT_COUNT" "$seed" "$robot" >"$WORKDIR/inject_$tag.json" 2>/dev/null
  if "$PORT" "$SEED_BASE" "$TIMEOUT_SECONDS" collision dense \
       <"$WORKDIR/inject_$tag.json" >"$WORKDIR/inject_$tag.ndjson" 2>"$WORKDIR/inject_$tag.err"; then
    echo "  PASS inject=collision $tag -- $(tail -1 "$WORKDIR/inject_$tag.err")"
  else
    echo "  FAIL inject=collision $tag did not reject every injected path:" >&2
    cat "$WORKDIR/inject_$tag.err" >&2
    failed+=("inject=collision $tag")
  fi
  # The same injected paths through upstream's checker. This is what turns
  # the injection gate from "two of this port's entry points agree" into
  # "the state parry calls a collision, FCL calls a collision too" -- and it
  # is also the proof that the cross-check on the real paths below can fail,
  # since a checker that answered `valid` unconditionally would fail here.
  oracle_path_check "inject_$tag" "$robot" "$WORKDIR/inject_$tag.json" \
    "$WORKDIR/inject_$tag.ndjson" invalid ""

  # Positive control: same scene, same problems, no injection. Without it the
  # rejection above proves nothing -- a checker that rejects everything would
  # also "pass" it.
  if "$PORT" "$SEED_BASE" "$TIMEOUT_SECONDS" "" dense \
       <"$WORKDIR/inject_$tag.json" >"$WORKDIR/control_$tag.ndjson" 2>"$WORKDIR/control_$tag.err"; then
    cp=$(port_aggregate "$WORKDIR/control_$tag.ndjson" \
          | jq -r '"\(.condition2_pass)/\(.condition2_checked)"')
    if [[ "$cp" == */* && "${cp%/*}" == "${cp#*/}" && "${cp%/*}" != "0" ]]; then
      echo "  PASS no-injection control $tag -- condition2 $cp"
    else
      echo "  FAIL no-injection control $tag -- condition2 $cp (expected all-pass)" >&2
      failed+=("control $tag")
    fi
  else
    echo "  FAIL no-injection control $tag did not run" >&2
    failed+=("control $tag")
  fi
done

# The constraint half of the injection gate needs a request that carries a
# `joint_constraint`, which only the constrained set does.
"$GEN" floor_wall "$INJECT_COUNT" 900011 panda panda_joint1:0.0:0.5 \
  >"$WORKDIR/inject_con.json" 2>/dev/null
if "$PORT" "$SEED_BASE" "$TIMEOUT_SECONDS" constraint dense \
     <"$WORKDIR/inject_con.json" >"$WORKDIR/inject_con.ndjson" 2>"$WORKDIR/inject_con.err"; then
  echo "  PASS inject=constraint -- $(tail -1 "$WORKDIR/inject_con.err")"
else
  echo "  FAIL inject=constraint did not reject every injected path:" >&2
  cat "$WORKDIR/inject_con.err" >&2
  failed+=("inject=constraint")
fi
oracle_path_check inject_constraint panda "$WORKDIR/inject_con.json" \
  "$WORKDIR/inject_con.ndjson" invalid panda_joint1:0.0:0.5

# --- the benchmark sets ----------------------------------------------------
#
# Generation is serial and cheap (<1s each); the port runs are the cost, so
# they go in parallel -- one process per set. Each problem is independent
# (fresh PlanningScene, seed = SEED_BASE + problem.id), so running sets
# concurrently changes no result, only the wall clock.
echo
echo "=== generating problem sets (count=$COUNT per config, mode=$MODE) ==="
for entry in "${SETS[@]}"; do
  read -r robot config count seed <<<"$entry"
  tag="${robot}_${config}"
  "$GEN" "$config" "$count" "$seed" "$robot" \
    >"$WORKDIR/$tag.request.json" 2>"$WORKDIR/$tag.gen.stderr" \
    || { echo "FAIL generating $tag" >&2; failed+=("gen $tag"); continue; }
  echo "  $(cat "$WORKDIR/$tag.gen.stderr")"
done

read -r c_robot c_config c_count c_seed c_spec <<<"$CONSTRAINED_SET"
"$GEN" "$c_config" "$c_count" "$c_seed" "$c_robot" "$c_spec" \
  >"$WORKDIR/constrained.request.json" 2>"$WORKDIR/constrained.gen.stderr" \
  || { echo "FAIL generating constrained set" >&2; failed+=("gen constrained"); }
echo "  $(cat "$WORKDIR/constrained.gen.stderr")"

echo
echo "=== C++ OMPL RRTConnect baseline (oracle) ==="
for entry in "${SETS[@]}"; do
  read -r robot config count seed <<<"$entry"
  tag="${robot}_${config}"
  [[ -s "$WORKDIR/$tag.request.json" ]] || continue
  # Redirected to a file, never piped into a filter: a pipe reports the
  # filter's status, which turns an oracle failure into a silent pass
  # (`verify-oracle-sweep.sh` makes the same point).
  if ! oracle_call "$ORACLE_BASELINE_TIMEOUT" -- \
       sg docker -c "$ORACLE --urdf $REPO_ROOT/fixtures/$robot.urdf --srdf $REPO_ROOT/fixtures/$robot.srdf" \
       <"$WORKDIR/$tag.request.json" >"$WORKDIR/$tag.oracle.json" 2>"$WORKDIR/$tag.oracle.stderr"; then
    oracle_call_explain "$ORACLE_CALL_STATUS" "oracle $tag: "
    tail -5 "$WORKDIR/$tag.oracle.stderr" >&2
    failed+=("oracle $tag")
    continue
  fi
  # `ok` before anything is read off `result`: the oracle answers a request it
  # could not serve with `{ok:false, error}` and exit 0 (one process serves a
  # stream of requests, so one bad request is not fatal to it). Left unchecked,
  # that lands as a *lower* C++ success rate -- and condition 1's bar is
  # `0.9 * cpp_rate`, so a broken baseline makes the port easier to pass.
  if [[ "$(jq -r '.ok' "$WORKDIR/$tag.oracle.json")" != "true" ]]; then
    echo "FAIL oracle answered not-ok for $tag: $(jq -r '.error' "$WORKDIR/$tag.oracle.json")" >&2
    failed+=("oracle not-ok $tag")
    continue
  fi
  n=$(jq '.result.problems | length' "$WORKDIR/$tag.oracle.json")
  ex=$(jq '[.result.problems[]|select(.exact==true)]|length' "$WORKDIR/$tag.oracle.json")
  echo "  $tag: $ex/$n exact"
done

echo
echo "=== port (cspace-planners-sbp rrt_connect), timeout=${TIMEOUT_SECONDS}s per call, ${SHARDS} shards ==="
port_start=$(date +%s.%N)
for entry in "${SETS[@]}" "constrained x x x"; do
  read -r robot config count seed <<<"$entry"
  tag="${robot}_${config}"
  [[ "$robot" == "constrained" ]] && tag="constrained"
  [[ -s "$WORKDIR/$tag.request.json" ]] || continue
  # `dense`: every solved path carries the waypoint list condition 2 checked,
  # so `oracle_path_check` can put the same states in front of upstream's own
  # checker below. Not sampled -- condition 2's wording is "100% of produced
  # paths", and a sample would answer a narrower question than that.
  if ! run_port_sharded "$WORKDIR/$tag.request.json" "$WORKDIR/$tag.port.ndjson" \
       "$TIMEOUT_SECONDS" dense; then
    echo "FAIL port run for $tag" >&2
    failed+=("port $tag")
  fi
  # Every aggregate below reads this projection instead of the run's own file:
  # with `dense` on, one full set is tens of MB of waypoints, and each of the
  # six later jq passes would re-parse all of them to reach the same handful
  # of scalars. Only `oracle_path_check` needs the waypoints themselves.
  jq -c 'select(.id != null) | del(.dense)' \
    "$WORKDIR/$tag.port.ndjson" >"$WORKDIR/$tag.slim.ndjson"
  port_aggregate "$WORKDIR/$tag.slim.ndjson" | jq -r --arg tag "$tag" \
    '"  \($tag): problems=\(.problems) solved=\(.solved) timeouts=\(.timeouts) failures=\(.failures) cond2=\(.condition2_pass)/\(.condition2_checked) waypoints=\(.waypoints_checked) (raw \(.raw_waypoints)) max_endpoint_gap=\(.max_endpoint_gap) cpu=\(.cpu_seconds|floor)s slowest=\((.slowest_seconds*10|round)/10)s (id \(.slowest_problem_id))"'
done
port_end=$(date +%s.%N)
port_wall=$(echo "$port_end - $port_start" | bc)
printf "  wall clock for all port runs: %.1fs\n" "$port_wall"

# --- condition 2 against upstream's checker, on every produced path ---------
#
# The injection gate above proved this comparison can fail; this is the
# comparison itself. Nothing here is sampled: every path condition 2 counted
# is handed to upstream `PlanningScene::isPathValid` and must come back valid,
# with the same (empty) invalid-index set.
echo
echo "=== condition 2 cross-check: every produced path through upstream isPathValid ==="
for entry in "${SETS[@]}" "constrained x x x"; do
  read -r robot config count seed <<<"$entry"
  tag="${robot}_${config}"
  spec=""
  if [[ "$robot" == "constrained" ]]; then
    tag="constrained"
    robot="$c_robot"
    spec="$c_spec"
  fi
  [[ -s "$WORKDIR/$tag.port.ndjson" ]] || continue
  oracle_path_check "$tag" "$robot" "$WORKDIR/$tag.request.json" \
    "$WORKDIR/$tag.port.ndjson" valid "$spec"
done

# --- feasibility accounting ------------------------------------------------
#
# A success rate over an unknown mix of feasible and infeasible problems is
# not a measurement of the planner. The endpoint filter in
# `plan_benchmark_problem_set.rs` guarantees both endpoints are valid, which
# is necessary for solvability but not sufficient -- an obstacle can separate
# two individually-valid endpoints into disconnected components of the free
# space, and no planner can join those.
#
# What is decidable here, and what is not:
#
#   * A returned path that passes the collision/constraint check IS a
#     constructive proof the problem is solvable. Every problem either side
#     solved is therefore "feasible, witnessed".
#   * No planner failure proves infeasibility. Sampling-based planners are
#     probabilistically complete: they find a path eventually IF one exists,
#     with no finite budget at which failure becomes proof. So the remainder
#     is reported as "feasibility unknown", never as "infeasible".
#
# The remainder is re-run at an escalated budget on both sides to convert as
# many unknowns to witnessed as possible. What still fails is reported as
# unknown, with its count -- that count is the honest denominator caveat on
# condition 1, not a footnote to omit.
#
# The escalated budget is mode-dependent, and the reason is a measured one: at
# 50000 iterations with a 600 s deadline, escalating fanuc's handful of
# unsolved problems took longer than the entire rest of the script, because
# these are exactly the problems where the planner does not converge and so
# every one of them runs to its full deadline. The pilot therefore escalates
# cheaply -- enough to exercise this code path, which is the point of a pilot
# -- and only `full` pays for the real attempt. The budget actually used is
# printed and recorded, so a pilot's feasibility count is never mistaken for
# the gate's.
if [[ "$MODE" == "full" ]]; then
  ESCALATED_ITERATIONS=50000
  ESCALATED_TIMEOUT=300
else
  ESCALATED_ITERATIONS=10000
  ESCALATED_TIMEOUT=30
fi

echo
echo "=== feasibility accounting (escalated to ${ESCALATED_ITERATIONS} iterations / ${ESCALATED_TIMEOUT}s for problems neither side solved) ==="
feas_rows="$WORKDIR/feasibility.json"
: >"$feas_rows"

# Which problems neither side solved, per set. Cheap, so serial.
declare -A UNSOLVED_N
for entry in "${SETS[@]}"; do
  read -r robot config count seed <<<"$entry"
  tag="${robot}_${config}"
  [[ -s "$WORKDIR/$tag.oracle.json" && -s "$WORKDIR/$tag.slim.ndjson" ]] || continue
  jq -s -r '
    (.[0].result.problems | map(select(.exact==true) | .id)) as $c |
    (.[1] | map(select(.outcome=="solved") | .id)) as $p |
    (.[0].result.problems | map(.id)) as $all |
    ($all - ($c + $p)) | .[]
  ' "$WORKDIR/$tag.oracle.json" <(jq -s '.' "$WORKDIR/$tag.slim.ndjson") \
    >"$WORKDIR/$tag.unsolved.txt" 2>/dev/null
  UNSOLVED_N[$tag]=$(wc -l <"$WORKDIR/$tag.unsolved.txt")
done

# The escalated runs are the expensive part -- every problem here is one that
# already failed once, so most run to their full deadline. Sharded for the
# same reason and by the same exact-equivalence argument as the benchmark
# runs above. The oracle side is not sharded: OMPL is ~24x faster per problem
# than the port here (measured), so it is not the cost.
for entry in "${SETS[@]}"; do
  read -r robot config count seed <<<"$entry"
  tag="${robot}_${config}"
  [[ "${UNSOLVED_N[$tag]:-0}" -gt 0 ]] || continue
  jq --argjson ids "$(jq -R -s 'split("\n")|map(select(length>0)|tonumber)' <"$WORKDIR/$tag.unsolved.txt")" \
     --argjson it "$ESCALATED_ITERATIONS" \
     '.max_iterations=$it | .problems = [.problems[]|select(.id as $i | $ids|index($i))]' \
     "$WORKDIR/$tag.request.json" >"$WORKDIR/$tag.escalate.json"
  # Previously bare with stderr discarded to /dev/null and no status check at
  # all -- ANY failure here, not just a hang, was silently invisible: the
  # unsolved-count accounting below just read whatever this call happened to
  # leave in `$tag.escalate.oracle.json`, empty or not. `oracle_call` cannot
  # surface a bound firing as a distinct, named failure into a status nothing
  # reads, so this now checks it -- bringing this call in line with every
  # other oracle call in this file, none of which discard their diagnosis.
  if ! oracle_call "$ORACLE_ESCALATION_TIMEOUT" -- \
       sg docker -c "$ORACLE --urdf $REPO_ROOT/fixtures/$robot.urdf --srdf $REPO_ROOT/fixtures/$robot.srdf" \
       <"$WORKDIR/$tag.escalate.json" >"$WORKDIR/$tag.escalate.oracle.json" 2>"$WORKDIR/$tag.escalate.oracle.stderr"; then
    oracle_call_explain "$ORACLE_CALL_STATUS" "escalation $tag (oracle): "
    tail -5 "$WORKDIR/$tag.escalate.oracle.stderr" >&2
    failed+=("escalation $tag (oracle)")
  fi
  run_port_sharded "$WORKDIR/$tag.escalate.json" "$WORKDIR/$tag.escalate.port.ndjson" \
    "$ESCALATED_TIMEOUT" || failed+=("escalation $tag")
done

for entry in "${SETS[@]}"; do
  read -r robot config count seed <<<"$entry"
  tag="${robot}_${config}"
  [[ -s "$WORKDIR/$tag.oracle.json" && -s "$WORKDIR/$tag.slim.ndjson" ]] || continue
  n_unsolved="${UNSOLVED_N[$tag]:-0}"
  n_total=$(jq '.result.problems|length' "$WORKDIR/$tag.oracle.json")
  witnessed=$((n_total - n_unsolved))
  still_unknown=0

  unknown_clock=0
  escalate_slowest=null
  if [[ "$n_unsolved" -gt 0 ]]; then
    # One jq pass computes the union AND the still-unknown breakdown from the
    # same `$still_unknown_ids`, rather than a second pass re-deriving that
    # set: two derivations of "which ids are still unknown" could disagree,
    # and a disagreement here would be invisible (both are just counts).
    #
    # The breakdown matters because `still_unknown` conflates two different
    # facts under one number. The port's own `outcome` field already tells
    # them apart per problem -- `"timeout"` is `PlanningFailure::
    # DeadlineExhausted` (`rrt_connect.rs`), the port's *clock* firing before
    # its search did; `"iterations_exhausted"`/`"invalid_endpoint"` are the
    # search itself running out of budget or never starting. The oracle's
    # `plan()` op offers OMPL only an iteration-counting
    # `PlannerTerminationCondition` (`oracle.cpp:6068-6069`, no wall clock at
    # either stage), so it has no clock-stop of its own to confuse with a
    # search failure. `still_unknown_port_clock_bound` is therefore the
    # subset of `still_unknown` that says "the port's escalated deadline cut
    # this off," not "the port could not find a path" -- see
    # `escalate_port_slowest_seconds` below for whether that deadline is
    # anywhere near binding.
    read -r e_cpp e_port newly still_unknown unknown_clock escalate_slowest < <(
      jq -nr \
        --slurpfile o "$WORKDIR/$tag.escalate.oracle.json" \
        --slurpfile p <(jq -s '.' "$WORKDIR/$tag.escalate.port.ndjson") \
        --argjson n_unsolved "$n_unsolved" '
      (($o[0].result.problems//[])|map(select(.exact==true)|.id)) as $cpp_ids |
      (($p[0]//[])|map(select(.outcome=="solved")|.id)) as $port_ids |
      # Union by id, not a sum: a problem solved by both sides at the
      # escalated budget is one witness, and adding the two counts would
      # over-credit it and could report more newly-witnessed problems than
      # were escalated.
      (($cpp_ids + $port_ids)|unique) as $newly_ids |
      (($p[0]//[])|map(.id)) as $escalated_ids |
      ($escalated_ids - $newly_ids) as $still_unknown_ids |
      (($p[0]//[])|map(select(.id as $i|$still_unknown_ids|index($i)))) as $rows |
      [ ($cpp_ids|length), ($port_ids|length), ($newly_ids|length),
        ($still_unknown_ids|length),
        ([$rows[]|select(.outcome=="timeout")]|length),
        (([$p[0][]|.plan_seconds]|max) // 0)
      ] | @tsv' 2>/dev/null
    )
    e_cpp="${e_cpp:-0}"; e_port="${e_port:-0}"; newly="${newly:-0}"
    still_unknown="${still_unknown:-$n_unsolved}"
    unknown_clock="${unknown_clock:-0}"
    escalate_slowest="${escalate_slowest:-null}"
    # More newly-witnessed than were escalated is arithmetically impossible;
    # if it happens the escalation read results that are not this set's (the
    # shape of the shard-leftover defect `run_port_sharded` now prevents).
    # Fail loudly rather than publish a negative unknown count.
    if [[ "$newly" -gt "$n_unsolved" ]]; then
      echo "FAIL $tag: escalation witnessed $newly of $n_unsolved escalated problems" >&2
      failed+=("escalation accounting $tag")
      newly="$n_unsolved"
      still_unknown=0
      unknown_clock=0
    fi
    witnessed=$((witnessed + newly))
    echo "  $tag: $n_unsolved unsolved at budget -> escalated (C++ $e_cpp, port $e_port, union $newly newly witnessed), $still_unknown still unknown ($unknown_clock port-clock-bound of ${ESCALATED_TIMEOUT}s, escalated slowest call ${escalate_slowest}s)"
  else
    echo "  $tag: every problem witnessed feasible at the benchmark budget"
  fi

  jq -n --arg tag "$tag" --argjson total "$n_total" \
        --argjson witnessed "$witnessed" --argjson unknown "$still_unknown" \
        --argjson unknown_clock "$unknown_clock" \
        --argjson escalate_slowest "$escalate_slowest" \
    '{tag:$tag, problems:$total, feasible_witnessed:$witnessed, feasibility_unknown:$unknown,
      feasibility_unknown_port_clock_bound:$unknown_clock,
      escalate_port_slowest_seconds:$escalate_slowest}' \
    >>"$feas_rows"
done
jq -s '.' "$feas_rows" >"$WORKDIR/feasibility_array.json"

# --- the three conditions --------------------------------------------------
#
# Computed by jq from the two result files rather than by reading a number
# off a log: conditions 1 and 3 are ratios between the C++ and port sets, and
# a ratio assembled by hand from two reports is exactly where a transcription
# error lands.
echo
echo "=== Phase 7 completion conditions ==="

summarize() {
  # $1 tag, $2 robot, $3 config, $4 seed. Emits one JSON object combining the
  # oracle results with `port_aggregate`'s recomputed port-side numbers. The
  # robot/config/seed come from the caller's own SETS entry rather than from
  # inside a result file, so a mislabelled result cannot rename itself.
  local tag="$1" robot="$2" config="$3" seed="$4"
  jq -n \
    --arg tag "$tag" --arg robot "$robot" --arg config "$config" \
    --argjson seed "$seed" --argjson seed_base "$SEED_BASE" \
    --argjson timeout "$TIMEOUT_SECONDS" \
    --slurpfile oracle "$WORKDIR/$tag.oracle.json" \
    --slurpfile port <(port_aggregate "$WORKDIR/$tag.slim.ndjson") \
    --slurpfile rows <(jq -s 'map(select(.id != null))' "$WORKDIR/$tag.slim.ndjson") \
    --slurpfile req "$WORKDIR/$tag.request.json" '
    def median: sort | if length==0 then null
      elif (length%2)==1 then .[length/2|floor]
      else (.[length/2-1]+.[length/2])/2 end;
    ($oracle[0].result.problems) as $c |
    ($port[0]) as $s |
    ([$c[]|select(.exact==true)]|length) as $c_solved |
    ($c|length) as $c_n |
    # Condition 3 as written compares two medians taken over each side`s OWN
    # solved set, and those are different populations: a port that fails the
    # hard problems drops their long paths out of its own median and passes a
    # 1.3x limit more easily the worse it gets. The paired median is over the
    # problems BOTH sides solved -- the same population on both sides, so a
    # ratio computed from it cannot be improved by failing anything. At
    # panda`s ~99% rates the two are nearly the same statistic; at fanuc`s
    # ~81% they need not be.
    ([$c[]|select(.exact==true)|.id]) as $c_ids |
    # What each problem asked for, by id, so the C++ path can be held to the
    # same endpoint requirement as the port path. Without this the two sides
    # are checked asymmetrically: a baseline whose paths do not start where the
    # problem said would still set condition 1`s bar and condition 3`s limit.
    # `length` on a number is jq`s absolute value. An `exact` problem with an
    # empty path makes the subtraction error out, which fails `summarize` for
    # this set -- a skip would be the same hole in a quieter form.
    ($req[0].problems | map({key: (.id|tostring), value: {start, goal}})
                      | from_entries) as $asked |

    ([$rows[0][]|select(.outcome=="solved")|.id]) as $p_ids |
    (($c_ids - ($c_ids - $p_ids))|sort) as $both |
    {
      tag: $tag, robot: $robot, config: $config,
      seed: $seed, seed_base: $seed_base, timeout_seconds: $timeout,
      problems: $c_n,
      cpp_solved: $c_solved,
      cpp_rate: (if $c_n>0 then $c_solved/$c_n else null end),
      cpp_median_length: ([$c[]|select(.exact==true)|.length]|median),
      cpp_max_endpoint_gap:
        ([$c[] | select(.exact==true) | . as $p
          | $asked[$p.id|tostring] as $q
          | [ ($q.start|to_entries[]) | (.value - $p.path[0][.key]) | length ]
          + [ ($q.goal |to_entries[]) | (.value - $p.path[-1][.key]) | length ]]
         | flatten | max // 0),
      port_problems: $s.problems,
      port_solved: $s.solved,
      port_timeouts: $s.timeouts,
      port_failures: $s.failures,
      port_rate: (if $s.problems>0 then $s.solved/$s.problems else null end),
      port_median_length: $s.median_length,
      paired_problems: ($both|length),
      cpp_paired_median: ([$c[]|select(.exact==true and (.id as $i|$both|index($i)))|.length]|median),
      port_paired_median: ([$rows[0][]|select(.outcome=="solved" and (.id as $i|$both|index($i)))|.length]|median),
      condition2_checked: $s.condition2_checked,
      condition2_pass: $s.condition2_pass,
      waypoints_checked: $s.waypoints_checked,
      raw_waypoints: $s.raw_waypoints,
      max_endpoint_gap: $s.max_endpoint_gap,
      cpu_seconds: $s.cpu_seconds,
      slowest_seconds: $s.slowest_seconds,
      slowest_problem_id: $s.slowest_problem_id
    }
    # The two sides must have been asked the same questions. A shard that
    # died, or a request the oracle partly rejected, would otherwise show up
    # as a success-rate difference rather than as the plumbing fault it is.
    | if .problems != .port_problems then
        error("\($tag): oracle saw \(.problems) problems but port saw \(.port_problems)")
      else . end'
}

per_set="$WORKDIR/per_set.json"
: >"$per_set"
for entry in "${SETS[@]}"; do
  read -r robot config count seed <<<"$entry"
  tag="${robot}_${config}"
  [[ -s "$WORKDIR/$tag.oracle.json" && -s "$WORKDIR/$tag.slim.ndjson" ]] || continue
  if ! summarize "$tag" "$robot" "$config" "$seed" >>"$per_set"; then
    echo "FAIL summarizing $tag" >&2
    failed+=("summarize $tag")
  fi
done

jq -s '.' "$per_set" >"$WORKDIR/per_set_array.json"

# The gate set is panda's two configs: 500 problems at FULL_COUNT, the set
# §5 names and the one `benches/sweep_baseline.sh` validated the difficulty
# of at n=250. fanuc's two configs are reported alongside as second-robot
# coverage, not folded into the gate -- a pass averaged across robots could
# hide a robot that fails.
verdict_json="$WORKDIR/verdict.json"
jq --arg mode "$MODE" '
  def median: sort | if length==0 then null
    elif (length%2)==1 then .[length/2|floor]
    else (.[length/2-1]+.[length/2])/2 end;
  def agg($rows):
    ($rows|map(.problems)|add // 0) as $n |
    ($rows|map(.cpp_solved)|add // 0) as $cs |
    ($rows|map(.port_solved)|add // 0) as $ps |
    {
      problems: $n,
      cpp_solved: $cs, cpp_rate: (if $n>0 then $cs/$n else null end),
      port_solved: $ps, port_rate: (if $n>0 then $ps/$n else null end),
      port_timeouts: ($rows|map(.port_timeouts)|add // 0),
      port_failures: ($rows|map(.port_failures)|add // 0),
      condition2_checked: ($rows|map(.condition2_checked)|add // 0),
      condition2_pass: ($rows|map(.condition2_pass)|add // 0),
      waypoints_checked: ($rows|map(.waypoints_checked)|add // 0),
      raw_waypoints: ($rows|map(.raw_waypoints)|add // 0),
      max_endpoint_gap: ($rows|map(.max_endpoint_gap // 0)|max // 0),
      cpp_max_endpoint_gap: ($rows|map(.cpp_max_endpoint_gap // 0)|max // 0),
      slowest_seconds: ($rows|map(.slowest_seconds)|max)
    };
  {
    mode: $mode,
    gate: (agg([.[]|select(.robot=="panda")])),
    fanuc: (agg([.[]|select(.robot=="fanuc")])),
    per_set: .
  }
  | .gate.cpp_median_length = ([.per_set[]|select(.robot=="panda")|.cpp_median_length]|median)
  | .gate.port_median_length = ([.per_set[]|select(.robot=="panda")|.port_median_length]|median)
  | .fanuc.cpp_median_length = ([.per_set[]|select(.robot=="fanuc")|.cpp_median_length]|median)
  | .fanuc.port_median_length = ([.per_set[]|select(.robot=="fanuc")|.port_median_length]|median)
' "$WORKDIR/per_set_array.json" >"$verdict_json"

# Per-set medians averaged into a set-of-sets median is not the same statistic
# as the median over the pooled problems, and §5 names the latter. Recompute
# from the pooled per-problem values so the reported number is the one the
# condition actually asks for -- and compute the paired pooled median beside
# it, over the problems both sides solved (see `summarize`'s own note on why a
# median over each side's own solved set can be improved by failing more).
#
# Keyed `tag#id`, never by id alone: every set numbers its problems from 0, so
# pooling two sets by bare id would pair `floor_wall` problem 7 with `cage`
# problem 7.
#
# $1 stratum name, then its tags. Emits one JSON object.
pooled_medians() {
  local stratum="$1"
  shift
  local oc="$WORKDIR/pooled.$stratum.cpp.ndjson" pc="$WORKDIR/pooled.$stratum.port.ndjson"
  : >"$oc"
  : >"$pc"
  local tag any=0
  for tag in "$@"; do
    [[ -s "$WORKDIR/$tag.oracle.json" && -s "$WORKDIR/$tag.slim.ndjson" ]] || continue
    any=1
    jq -c --arg tag "$tag" \
      '.result.problems[] | {key: ($tag + "#" + (.id|tostring)), exact, length}' \
      "$WORKDIR/$tag.oracle.json" >>"$oc"
    jq -c --arg tag "$tag" \
      '{key: ($tag + "#" + (.id|tostring)), outcome, length}' \
      "$WORKDIR/$tag.slim.ndjson" >>"$pc"
  done
  if [[ "$any" == "0" ]]; then
    echo 'null'
    return 0
  fi
  jq -n --slurpfile c <(jq -s '.' "$oc") --slurpfile p <(jq -s '.' "$pc") '
    def median: sort | if length==0 then null
      elif (length%2)==1 then .[length/2|floor]
      else (.[length/2-1]+.[length/2])/2 end;
    ($c[0]) as $cp | ($p[0]) as $pt |
    ([$cp[]|select(.exact==true)|.key]) as $ck |
    ([$pt[]|select(.outcome=="solved")|.key]) as $pk |
    (($ck - ($ck - $pk))|unique) as $both |
    {
      cpp_median_length_pooled: ([$cp[]|select(.exact==true)|.length]|median),
      port_median_length_pooled: ([$pt[]|select(.outcome=="solved")|.length]|median),
      paired_problems_pooled: ($both|length),
      cpp_paired_median_pooled: ([$cp[]|select(.exact==true and (.key as $k|$both|index($k)))|.length]|median),
      port_paired_median_pooled: ([$pt[]|select(.outcome=="solved" and (.key as $k|$both|index($k)))|.length]|median)
    }'
}

pooled_gate="$(pooled_medians gate panda_floor_wall panda_cage)"
pooled_fanuc="$(pooled_medians fanuc fanuc_floor_wall fanuc_cage)"

jq --argjson pooled_gate "${pooled_gate:-null}" \
   --argjson pooled_fanuc "${pooled_fanuc:-null}" \
   --argjson wall "$port_wall" \
   --slurpfile feas "$WORKDIR/feasibility_array.json" \
   '.gate += ($pooled_gate // {})
    | .fanuc += ($pooled_fanuc // {})
    | .parallel_wall_clock_seconds=$wall
    | .feasibility = $feas[0]
    | .feasibility_escalated_iterations = '"$ESCALATED_ITERATIONS"'
    | .feasibility_escalated_timeout_seconds = '"$ESCALATED_TIMEOUT"'
    | .benchmark_timeout_seconds = '"$TIMEOUT_SECONDS"'
    | .problems_per_config = '"$COUNT"'
    | .gate.feasible_witnessed = ([$feas[0][]|select(.tag|startswith("panda"))|.feasible_witnessed]|add // 0)
    | .gate.feasibility_unknown = ([$feas[0][]|select(.tag|startswith("panda"))|.feasibility_unknown]|add // 0)
    | .gate.feasibility_unknown_port_clock_bound = ([$feas[0][]|select(.tag|startswith("panda"))|.feasibility_unknown_port_clock_bound]|add // 0)
    | .gate.escalate_port_slowest_seconds = ([$feas[0][]|select(.tag|startswith("panda"))|.escalate_port_slowest_seconds]|max // null)
    | .fanuc.feasible_witnessed = ([$feas[0][]|select(.tag|startswith("fanuc"))|.feasible_witnessed]|add // 0)
    | .fanuc.feasibility_unknown = ([$feas[0][]|select(.tag|startswith("fanuc"))|.feasibility_unknown]|add // 0)
    | .fanuc.feasibility_unknown_port_clock_bound = ([$feas[0][]|select(.tag|startswith("fanuc"))|.feasibility_unknown_port_clock_bound]|add // 0)
    | .fanuc.escalate_port_slowest_seconds = ([$feas[0][]|select(.tag|startswith("fanuc"))|.escalate_port_slowest_seconds]|max // null)' \
   "$verdict_json" >"$verdict_json.tmp" && mv "$verdict_json.tmp" "$verdict_json"

# The constrained set carries condition 2's constraint half; it has no C++
# counterpart, so it is folded in as its own stratum rather than through
# `summarize`.
con=$(port_aggregate "$WORKDIR/constrained.slim.ndjson" 2>/dev/null \
      | jq --arg spec "$c_spec" '. + {joint_constraint: $spec}')

jq -r '
  .gate as $g |
  "  problems (gate set, panda): \($g.problems)",
  "  feasibility: \($g.feasible_witnessed) witnessed solvable (a validated path exists), \($g.feasibility_unknown) unknown after escalation (\($g.feasibility_unknown_port_clock_bound) of those are the ports escalated deadline firing, not a search failure -- never called infeasible -- no finite budget proves that)",
  "  C++ OMPL RRTConnect: \($g.cpp_solved)/\($g.problems) = \((($g.cpp_rate//0)*10000|round)/100)%",
  "  port:                \($g.port_solved)/\($g.problems) = \((($g.port_rate//0)*10000|round)/100)%  (timeouts \($g.port_timeouts), other failures \($g.port_failures))",
  "  second robot (fanuc), its own stratum, never averaged into the gate set:",
  "    C++ \(.fanuc.cpp_solved)/\(.fanuc.problems) = \(((.fanuc.cpp_rate//0)*10000|round)/100)%, port \(.fanuc.port_solved)/\(.fanuc.problems) = \(((.fanuc.port_rate//0)*10000|round)/100)% (timeouts \(.fanuc.port_timeouts), other failures \(.fanuc.port_failures))",
  "    feasibility: \(.fanuc.feasible_witnessed) witnessed solvable, \(.fanuc.feasibility_unknown) unknown after escalation (\(.fanuc.feasibility_unknown_port_clock_bound) port-clock-bound, escalated slowest call \(.fanuc.escalate_port_slowest_seconds)s of '"$ESCALATED_TIMEOUT"'s -- the oracle side has no clock at either stage, so it has no counterpart figure)",
  "  slowest single planner call: \($g.slowest_seconds)s (panda), \(.fanuc.slowest_seconds)s (fanuc)",
  "  parallel wall clock, all port runs: \(.parallel_wall_clock_seconds)s",
  ""
' "$verdict_json"

# --- one decision list, printed and gated from the same place ---------------
#
# Each entry below is BOTH what gets printed and what decides the exit code.
# The previous shape computed every verdict twice -- once inside the reporting
# `jq -r`, once again beneath it as `c1=$(jq ...)` -- and carried a comment
# telling the next editor the two copies must match. They did not have to:
# fanuc's three conditions were printed with PASS/FAIL and never reached
# `failed`, so a second-robot regression printed FAIL and the script exited 0.
# One list makes that unrepresentable instead of something to keep in sync.
#
# `stratum` is a function rather than a written-out list because §5's wording
# names one population -- panda's 500 -- and that one supplies the completion
# verdict, but every other set measured here is a population too, and pooling
# or averaging populations is what lets a failure hide inside a pass. The rule
# is therefore: never average two strata, and gate each one under its own
# name. fanuc stays out of the panda numbers AND fails in its own name.
checks_json="$WORKDIR/checks.json"
jq -c --argjson con "${con:-null}" --argjson pins "$PINS_JSON" \
      --argjson tmo "$TIMEOUT_SECONDS" '
  def pct($x): (($x//0)*10000|round)/100;
  def r3($x): if $x == null then null else (($x)*1000|round)/1000 end;
  def ratio($num; $den): (if ($den//0) > 0 then ($num//0)/$den else null end);
  def stratum($s; $label; $pin):
    [
      { name: "\($label)/condition1",
        detail: "port \($s.port_solved)/\($s.problems) = \(pct($s.port_rate))% >= 0.9 x C++ \($s.cpp_solved)/\($s.problems), i.e. \(pct(($s.cpp_rate//0)*0.9))%",
        # `problems > 0` is not decoration: without it an empty set reads
        # 0 >= 0 and passes, which is how a harness that measured nothing
        # reports success.
        ok: (($s.problems//0) > 0 and ($s.port_rate//0) >= (($s.cpp_rate//0)*0.9)) },
      { name: "\($label)/condition2",
        detail: "\($s.condition2_pass)/\($s.condition2_checked) paths valid, \($s.waypoints_checked) waypoints checked",
        ok: (($s.condition2_checked//0) > 0 and $s.condition2_pass == $s.condition2_checked) },
      # A solved path whose validity was never checked is invisible to
      # condition 2 as it was written: `condition2_checked > 0` is satisfied
      # by one checked path out of five hundred.
      { name: "\($label)/condition2-covers-every-solved-path",
        detail: "checked \($s.condition2_checked) of \($s.port_solved) solved",
        ok: (($s.port_solved//0) > 0 and $s.condition2_checked == $s.port_solved) },
      # Densification is what makes condition 2 an independent check rather
      # than a re-reading of states the search already accepted, so a collapse
      # back to the returned vertices has to fail here, not report 100%.
      { name: "\($label)/condition2-densified",
        detail: "\($s.waypoints_checked) checked from \($s.raw_waypoints) returned",
        ok: (($s.raw_waypoints//0) > 0 and ($s.waypoints_checked//0) > ($s.raw_waypoints//0)) },
      # `solved` says a path came back, not that it is a path between the two
      # states that were asked for. Exact zero rather than a tolerance:
      # `rrt_connect` returns the endpoint states it was handed.
      { name: "\($label)/endpoints",
        detail: "max gap from requested start/goal over solved paths: \($s.max_endpoint_gap)",
        ok: (($s.max_endpoint_gap//1) == 0) },
      # The same requirement on the baseline. Condition 1`s bar is
      # `0.9 x cpp_rate` and condition 3`s limit is `1.3 x cpp_median`, so an
      # oracle whose `exact` paths do not run between the two states asked for
      # sets both of them from paths that answered a different question.
      { name: "\($label)/cpp-endpoints",
        detail: "max gap of the C++ path from the requested start/goal: \($s.cpp_max_endpoint_gap)",
        ok: (($s.cpp_max_endpoint_gap//1) == 0) },
      # A run containing a timeout is not reproducible -- the same tree on a
      # slower machine gives a different answer -- and condition 1 has enough
      # slack to absorb several without changing its verdict.
      { name: "\($label)/no-timeouts",
        detail: "\($s.port_timeouts) timeouts, slowest call \(r3($s.slowest_seconds))s of the \($tmo)s budget",
        ok: (($s.port_timeouts//1) == 0) },
      { name: "\($label)/condition3-pooled",
        detail: "\($s.port_median_length_pooled) vs limit \(($s.cpp_median_length_pooled//0)*1.3) (ratio \(r3(ratio($s.port_median_length_pooled; $s.cpp_median_length_pooled)))x)",
        ok: (($s.cpp_median_length_pooled//0) > 0 and ($s.port_median_length_pooled//0) <= ($s.cpp_median_length_pooled*1.3)) },
      # The same 1.3x over the problems BOTH sides solved. Condition 3 as
      # written takes each side median over its own solved set, and a port
      # that fails the hard problems drops their long paths out of its own
      # median -- passing more easily the worse it gets.
      { name: "\($label)/condition3-paired",
        detail: "over \($s.paired_problems_pooled) problems both sides solved: \($s.port_paired_median_pooled) vs limit \(($s.cpp_paired_median_pooled//0)*1.3) (ratio \(r3(ratio($s.port_paired_median_pooled; $s.cpp_paired_median_pooled)))x)",
        ok: (($s.paired_problems_pooled//0) > 0 and ($s.cpp_paired_median_pooled//0) > 0 and ($s.port_paired_median_pooled//0) <= ($s.cpp_paired_median_pooled*1.3)) }
    ]
    + (if $pin == null then [] else
      # See PINS_ALL for how each constant was sized and from which streams.
      [
      { name: "\($label)/pin-population",
        detail: "\($s.problems) problems, pinned at \($pin.problems)",
        ok: (($s.problems//0) == $pin.problems) },
      { name: "\($label)/no-regression-port-solved",
        detail: "port solved \($s.port_solved) >= floor \($pin.port_solved_floor)",
        ok: (($s.port_solved//0) >= $pin.port_solved_floor) },
      { name: "\($label)/no-regression-cpp-solved",
        detail: "C++ solved \($s.cpp_solved) >= floor \($pin.cpp_solved_floor)",
        ok: (($s.cpp_solved//0) >= $pin.cpp_solved_floor) },
      { name: "\($label)/no-regression-median-ratio",
        detail: "pooled ratio \(r3(ratio($s.port_median_length_pooled; $s.cpp_median_length_pooled)))x within [\($pin.ratio_floor), \($pin.ratio_ceiling)]",
        ok: ((ratio($s.port_median_length_pooled; $s.cpp_median_length_pooled) // 0) >= $pin.ratio_floor
             and (ratio($s.port_median_length_pooled; $s.cpp_median_length_pooled) // 99) <= $pin.ratio_ceiling) }
      ] end);
  # One set -> the two conditions that are ratios, at set granularity. Pooling
  # `floor_wall` with `cage` averages two populations just as much as
  # averaging panda with fanuc does, so each set is gated in its own name too.
  def per_set($row):
    [
      { name: "\($row.tag)/condition1",
        detail: "port \($row.port_solved)/\($row.problems) = \(pct($row.port_rate))% >= 0.9 x C++ \($row.cpp_solved)/\($row.problems), i.e. \(pct(($row.cpp_rate//0)*0.9))%",
        ok: (($row.problems//0) > 0 and ($row.port_rate//0) >= (($row.cpp_rate//0)*0.9)) },
      { name: "\($row.tag)/condition3",
        detail: "\($row.port_median_length) vs limit \(($row.cpp_median_length//0)*1.3) (ratio \(r3(ratio($row.port_median_length; $row.cpp_median_length)))x)",
        ok: (($row.cpp_median_length//0) > 0 and ($row.port_median_length//0) <= ($row.cpp_median_length*1.3)) }
    ];
  (
    stratum(.gate;  "panda(gate)"; $pins.panda)
    + stratum(.fanuc; "fanuc";     $pins.fanuc)
    + ([.per_set[] | per_set(.)] | add // [])
    + (if $con == null then [] else [
        { name: "constrained/condition2",
          detail: "constraint=\($con.joint_constraint), solved \($con.solved)/\($con.problems), \($con.condition2_pass)/\($con.condition2_checked) valid, \($con.waypoints_checked) waypoints",
          ok: (($con.condition2_checked//0) > 0 and $con.condition2_pass == $con.condition2_checked) },
        { name: "constrained/condition2-covers-every-solved-path",
          detail: "checked \($con.condition2_checked) of \($con.solved) solved",
          ok: (($con.solved//0) > 0 and $con.condition2_checked == $con.solved) },
        { name: "constrained/condition2-densified",
          detail: "\($con.waypoints_checked) checked from \($con.raw_waypoints) returned",
          ok: (($con.raw_waypoints//0) > 0 and ($con.waypoints_checked//0) > ($con.raw_waypoints//0)) },
        { name: "constrained/endpoints",
          detail: "max gap \($con.max_endpoint_gap)",
          ok: (($con.max_endpoint_gap//1) == 0) },
        { name: "constrained/no-timeouts",
          detail: "\($con.timeouts) timeouts",
          ok: (($con.timeouts//1) == 0) }
      ] end)
  )' "$verdict_json" >"$checks_json"

# Printed and gated in one pass over that same list. Process substitution, not
# a pipe: `failed+=` inside a piped `while` would land in a subshell and the
# exit code would lose every failure the loop printed.
while IFS=$'\t' read -r name ok detail; do
  if [[ "$ok" == "true" ]]; then
    printf '  PASS %-56s %s\n' "$name" "$detail"
  else
    printf '  FAIL %-56s %s\n' "$name" "$detail" >&2
    failed+=("$name")
  fi
done < <(jq -r '.[] | [.name, (.ok|tostring), .detail] | @tsv' "$checks_json")

# An empty list is not a pass. Every earlier stage appends to `failed` on its
# own faults, but a `jq` program that emitted nothing would otherwise print no
# lines and reach the exit as clean.
check_count="$(jq 'length' "$checks_json")"
if [[ "${check_count:-0}" -lt 1 ]]; then
  failed+=("no checks were evaluated")
fi

if [[ "$MODE" == "full" ]]; then
  # `dirty` is not decoration: a benchmark run from a working tree with
  # uncommitted changes was measured against code that `commit` does not
  # name, and a results file recording only the SHA would attribute the
  # numbers to a tree that never produced them. `dirty_paths` says *which*
  # files, so a later reader can decide whether the difference could touch
  # these numbers instead of having to guess from a bare boolean.
  # `$DIRTY_LIST` / `$TREE_DIRTY` were captured before the build alongside the
  # source digests, not here -- see that site for why.
  #
  # `commit` can only ever name the run's *parent* when the run is what
  # produces the artifact being committed, so on its own it cannot identify
  # the code that made these numbers -- and a note saying "the commit that
  # adds this harness" names a sha a later reader cannot resolve.
  #
  # `measured_sources` closes that by content instead of by revision: the
  # content digest of each source that determines a number here -- a file's own
  # git blob id, a directory's hash over every tracked file beneath it.
  # Checkable in one command against any tree, tracked or not, with no sha to
  # look up:
  #
  #   git hash-object crates/cspace-planners-sbp/examples/plan_benchmark_port.rs
  #
  # If that differs from the value recorded here, the committed code is not
  # the code that ran and these figures need re-measuring.
  #
  # `$SOURCES_JSON` was taken before the build, not here -- see the snapshot
  # site for why the timing is the point. `crates/cspace-planners-sbp/src` and
  # the oracle are in it because the harnesses are not what solves these
  # problems.
  jq --arg ts "$(date -u +%Y-%m-%dT%H:%M:%SZ)" \
     --arg stamp "$(cd "$REPO_ROOT" && git rev-parse HEAD)" \
     --arg oracle_stamp "$ORACLE_STAMP" \
     --argjson dirty "$TREE_DIRTY" \
     --argjson dirty_paths "$(printf '%s' "$DIRTY_LIST" | jq -R -s 'split("\n")|map(select(length>0))')" \
     --argjson sources "$SOURCES_JSON" \
     --argjson con "${con:-null}" \
     --argjson pins "$PINS_JSON" \
     --argjson failed "$(printf '%s\n' ${failed[@]+"${failed[@]}"} \
                         | jq -R -s 'split("\n")|map(select(length>0))')" \
     --slurpfile checks "$checks_json" \
     '{measured_at:$ts, commit:$stamp, oracle_stamp:$oracle_stamp, working_tree_dirty:$dirty,
       dirty_paths:$dirty_paths, measured_sources:$sources} + .
      + {constrained_set:$con, regression_pins:$pins,
         # `checks` keeps each detail string; `verdict` is the name -> PASS/FAIL
         # map. Neither is the list the exit code comes from: generation, the
         # port and oracle runs, the injection gate, its control, the
         # escalation pass and the cross-check all push to `failed` and none of
         # them is a `checks` entry. `verdict_all_pass` is taken from both, so
         # the file cannot disagree with the run that wrote it.
         checks:$checks[0],
         verdict:($checks[0]|map({key:.name,
                                  value:(if .ok then "PASS" else "FAIL" end)})
                           |from_entries),
         failed:$failed,
         verdict_all_pass:(($checks[0]|all(.ok)) and ($failed|length == 0))}' \
     "$verdict_json" >"$RESULTS"
  echo
  echo "  wrote $RESULTS"
else
  echo
  echo "  NOTE mode=pilot ran $PILOT_COUNT problems per config, NOT the 500 §5 names."
  echo "  NOTE these numbers are a harness self-test. They are not the Phase 7 gate."
  echo "  NOTE run '$0 full' for the measurement Phase 7's completion condition asks for."
fi

echo
if [[ ${#failed[@]} -gt 0 ]]; then
  echo "FAIL ${#failed[@]} Phase 7 check(s) failed:" >&2
  printf '  %s\n' "${failed[@]}" >&2
  exit 1
fi
# The count, not "conditions 1/2/3": the list gates each stratum and each set
# under its own name, so a message naming three conditions would understate
# what passed and, worse, would not move if a stratum stopped being checked.
echo "OK $check_count Phase 7 checks pass (mode=$MODE)"
