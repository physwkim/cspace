#!/bin/bash
# Phase 8's second completion condition -- "CHOMP/STOMP가 Phase 7과 같은 속성
# 기반 검증을 통과" (PORTING-PLAN.md §5) -- as a command rather than as a
# sentence.
#
# The condition names Phase 7's verification as its standard, so this script is
# derived item by item from what `verify-phase7-benchmark.sh` actually does, not
# from Phase 7's three conditions as prose. Several of those items do not
# transfer to an optimizing planner that starts from a seed trajectory instead
# of sampling; each one that does not is named, with its reason and its
# analogue, in PORTING-PLAN.md's Phase 8 property section, and the ones that do
# transfer are the checks below. In particular:
#
#   * The C++ baseline IS built here, and it is each planner's own upstream
#     implementation rather than C++ OMPL RRTConnect. The oracle answers
#     `chomp_plan` (`oracle.cpp`'s `chompPlan`, upstream `ChompPlanner::solve`)
#     and `stomp_plan` (`stompPlan`, upstream `StompPlanningContext::solve`),
#     and `measure-phase8-cpp-baseline.sh` drives them over the SAME request
#     files this port consumes, one oracle process per problem at the same
#     per-problem seed. So `condition1`, `condition3-pooled`,
#     `condition3-paired` and `no-regression-cpp-solved` are real checks here,
#     not analogues.
#
#     This is what changed the argument. A `0.9 x cpp_rate` bar against
#     RRTConnect measured the problem set: a local optimizer fails a sampler's
#     population by construction, and the bar would have been about algorithm
#     class. Against the same planner's own C++ code both sides fail the same
#     problems for the same reason, and what is left is the port. The pinned
#     `no-regression-solved` floor stays alongside it -- one watches this tree
#     against itself, the other against upstream, and neither subsumes the
#     other.
#   * `cpp-endpoints` still has no counterpart, and this one is not an
#     argument: `chompPlan` and `stompPlan` emit `solved`, `error_code`,
#     `length` and the condition-2 fields, but no `path`. Only the `plan` op
#     emits waypoints (`oracle.cpp:5870`), so there is nothing on the C++ side
#     to measure a start/goal gap from. Restoring it is an oracle SOURCE
#     change, which the digest gate turns into an image rebuild for every
#     worktree on this machine. See the plan section.
#   * The constrained population has no C++ baseline either, and for a
#     different reason: `chompPlan`/`stompPlan` never read the request's
#     `joint_constraint`, so the oracle would plan a different problem than
#     the port did. Phase 7 treats its own constrained set as condition-2-only
#     for the same reason.
#   * Path validity, its densification requirement, its coverage requirement,
#     the independent cross-check against upstream `isPathValid`, the injection
#     discrimination gate, the population pin, the per-stratum rule and the
#     no-timeout rule all transfer unchanged, and are checked below for both
#     planners.
#
# This does NOT change Phase 8's verdict word. It is the instrument that would
# let someone change it: `PLANNER=... MODE=full` produces the numbers, and
# `doc/phase8-optimizer-properties.json` records them with the blob ids of the
# files that produced them.
#
# # Why this is `measure-` and not `verify-`
#
# `verify-all.sh` runs every `tools/ci/verify-*.sh` by glob, per merge round,
# and `ci.yml` runs every `check-*.sh`. This file is deliberately in neither
# glob, which `verify-phase7-benchmark.sh` -- a glob member with a 312s pilot --
# is not. Two measured reasons, both about STOMP:
#
#   * Cost. Measured at PILOT_COUNT=8, seeds below, SEED_BASE=525252: the whole
#     gate takes 1326s wall clock, of which 712.7s is instrument time. The same
#     gate with `PLANNERS=chomp` takes 131s (45.7s instrument). Nearly all of
#     the difference is STOMP sitting on the 120s bound: 16 of its 40 problems
#     do, and the constrained set spends 8 x 120s to solve none of them. A
#     per-round glob member cannot absorb that, and CHOMP -- whose slowest call
#     in that run was 6.97s -- would be paying for it.
#   * A pilot cannot hold `no-timeouts` on this population. That same run fails
#     it on all four STOMP configs and on the constrained set. Those timeouts
#     are a real property -- see the plan section: a population of uniformly
#     sampled start/goal pairs in clutter is a *sampler's* population, and a
#     local optimizer over a straight-line seed fails a fraction of it by
#     construction. This file reports that number; it must not turn it into a
#     red round for everyone.
#
# So the run is explicit, and its measured wall clock is printed at the end of
# every run and recorded in the results file:
#
#   sg docker -c 'tools/ci/measure-phase8-optimizer-properties.sh full'
#
# docker is needed for one stage only, the cross-check against upstream
# `isPathValid`; it must go through `sg` because an unwrapped `docker` in this
# repo reports failure as success.

set -uo pipefail

MODE="${1:-pilot}"
case "$MODE" in
  pilot|full) ;;
  *) echo "usage: $0 [pilot|full]" >&2; exit 2 ;;
esac

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"

# This run's own code -- this file, the library it sources, the oracle digest
# helper -- copies itself into a private directory and re-execs from there,
# once, before doing anything else. Measured hazard, not a hypothetical one: a
# 95-minute run of this script survived a ten-commit `git merge` into the same
# worktree it was reading from only because the merge happened not to touch
# this file's bytes or a function this run had already sourced. Three distinct
# ways it could have gone wrong instead:
#   - corruption: an in-place rewrite of the file bash is still reading
#     mid-script (bash reads a script incrementally through one fd and byte
#     offset, so a rewrite under it is not atomic the way a `mv` would be)
#   - version split: `gate-lib.sh` sourced once at start, then edited on disk
#     mid-run -- the run keeps computing with the definitions it already read
#     while a later check validates against what the file holds now
#   - late swap: a helper this run has not reached yet (`measure-phase8-cpp-
#     baseline.sh`, invoked well into the run) or the `target/release/examples`
#     binaries, replaced before their turn, so one measurement is produced by
#     two versions of the instrument while the artifact names only one
#
# $REPO_ROOT itself is NOT redirected by this -- it keeps meaning the live
# tree for everything that measures or builds FROM it: `git status`/`git rev-
# parse`, `cargo build`, `fixtures/`, `doc/`, and every `measured_source_digest`
# call in $SOURCES_JSON below. Those have to describe and compile the real
# tree; only the CODE THAT RUNS has to stop reading it after this point. The
# built binaries are handled differently, by copy rather than by re-exec --
# see where `cargo build` finishes, below.
#
# `measure-phase8-cpp-baseline.sh` is not copied here. It carries the exact
# same hazard on its own invocation (a fresh `bash` reading it from
# $REPO_ROOT) and protects itself the same way, independently, at its own
# top, so it stays safe whether this script calls it or a person runs it
# directly -- see that file. `run-oracle.sh`, which both scripts invoke, is
# NOT snapshotted by either: it derives its own $REPO_ROOT from its own
# `$BASH_SOURCE[0]` and would need the same override this block adds, in a
# third file, to run from a copy correctly. That is out of this change's
# scope; a mid-run edit to run-oracle.sh's own dispatch logic (which image
# tag it resolves, which paths it mounts) is a real, named, NOT-covered
# exposure, distinct from the oracle image/source pin itself, which
# `oracle_stamp` already covers content-addressed and unaffected by this gap.
#
# `_PHASE8_OPT_REPO_ROOT` unset is how this code tells "reading from the live
# tree, first time" apart from "already running from the private copy" --
# not a caller-facing mode, nothing about how this script is invoked changes;
# it is the value that has to cross the `exec` boundary, because `exec`
# replaces this process's entire state and only the environment survives it.
if [[ -z "${_PHASE8_OPT_REPO_ROOT:-}" ]]; then
  WORKDIR="$(mktemp -d)"
  trap 'rm -rf "$WORKDIR"' EXIT
  SELF="$WORKDIR/self"
  # This file's own name, not a literal, so a rename cannot silently desync
  # the copy source from what this process was actually invoked as.
  SELF_NAME="$(basename "${BASH_SOURCE[0]}")"
  mkdir -p "$SELF/tools/ci" "$SELF/tools/moveit-oracle" || exit 1
  cp "$REPO_ROOT/tools/ci/$SELF_NAME" "$SELF/tools/ci/" || exit 1
  cp "$REPO_ROOT/tools/ci/gate-lib.sh" "$SELF/tools/ci/" || exit 1
  cp "$REPO_ROOT/tools/moveit-oracle/src-digest.sh" "$SELF/tools/moveit-oracle/" || exit 1
  export _PHASE8_OPT_REPO_ROOT="$REPO_ROOT"
  export _PHASE8_OPT_WORKDIR="$WORKDIR"
  exec "$SELF/tools/ci/$SELF_NAME" "$MODE"
fi

# Second execution, from the private copy. $REPO_ROOT is read back from the
# environment rather than re-derived from `$BASH_SOURCE[0]` here on purpose --
# re-deriving it would resolve to $SELF, the copy, and every git/cargo/
# fixtures/doc path below would silently point at a directory that does not
# hold them.
REPO_ROOT="$_PHASE8_OPT_REPO_ROOT"
WORKDIR="$_PHASE8_OPT_WORKDIR"
trap 'rm -rf "$WORKDIR"' EXIT

. "$(dirname "${BASH_SOURCE[0]}")/gate-lib.sh"

require_caller_tree "$REPO_ROOT"
cd "$REPO_ROOT" || exit 1  # no `set -e` here: a failed cd would
                           # otherwise run this gate in the caller's tree

ORACLE="$REPO_ROOT/tools/moveit-oracle/run-oracle.sh"
GEN="$REPO_ROOT/target/release/examples/plan_benchmark_problem_set"
CHOMP="$REPO_ROOT/target/release/examples/optimize_benchmark_chomp"
STOMP="$REPO_ROOT/target/release/examples/optimize_benchmark_stomp"
RESULTS="$REPO_ROOT/doc/phase8-optimizer-properties.json"

# Per-problem wall-clock bound. A call that hits it is a FAILURE, never a skip:
# see each instrument's own `DEFAULT_TIMEOUT_SECONDS` for how the two planners
# differ in what enforces it (STOMP is cancelled from outside, upstream's own
# mechanism; CHOMP sets its own `planning_time_limit` to this value, so its
# loop and this bound stop on one clock rather than two).
#
# It has to be far above any per-problem time either side needs, because the
# C++ baseline below is deliberately run at a 3600s clock so its ITERATION
# bound terminates: a port arm stopped by a tighter clock than the arm it is
# compared against reports the difference as a planner failure. CHOMP used to
# be, at `ChompParameters::default()`'s 6.0.
TIMEOUT_SECONDS=120

# Bound for oracle_path_check's is_state_valid round trip below. No search
# and no request-level clock bound -- a pure per-waypoint validity check has
# none to give. verify-phase7-benchmark.sh's own analogous call is directly
# measured at "~9s" for its small injection-gate batches; the condition-2
# cross-check below hands the same function every solved path from a whole
# config, up to hundreds. Sized well above either, and well below the
# escalation-sized bound that script's own retry stage needs, which is a
# structurally different (search, not validity-check) call.
ORACLE_PATH_CHECK_TIMEOUT="${ORACLE_PATH_CHECK_TIMEOUT:-600}"

# The C++ arm's clock, named here rather than left to
# `measure-phase8-cpp-baseline.sh`'s own default, so that both arms' stop
# conditions are set in one place and can be compared without reading two
# files. Both are written into `$RESULTS` for the same reason: the run that
# published `chomp/panda/condition1: FAIL` had a 6.0s port arm and a 3600s C++
# arm, and nothing in its own artifact said so.
#
# The two need not be equal, and are not: what matters is that neither is what
# stops a call, so that both arms terminate on their iteration bound. If one
# binds while the other does not, the rate difference is this instrument's
# artifact rather than a fact about the port. `no-timeouts` is the port arm's
# check for exactly that -- and it currently FAILS for STOMP (65/250 panda,
# 80/250 fanuc), so STOMP's condition-1 rates are not yet clean under this
# rule. CHOMP's `no-timeouts` passes.
CPP_CLOCK_BOUND=3600

# 125 per config x 4 configs = 500 problems per planner, the same population
# size §5 names for Phase 7. Phase 8's row names no count of its own, so this
# is this gate's declaration, and `pin-population` holds every run to it.
FULL_COUNT=125
# Overridable only because it cannot be used to fake coverage: `pin-population`
# fails a reduced run rather than reporting the smaller set as a pass. It
# exists so a mutation proof costs seconds instead of a whole pilot. The default
# is 8 because that is the count every pilot pin below was measured at.
PILOT_COUNT="${PILOT_COUNT:-8}"

if [[ "$MODE" == "full" ]]; then
  COUNT="$FULL_COUNT"
else
  COUNT="$PILOT_COUNT"
fi

# robot config count seed. The seeds live here, in the committed harness,
# rather than in a round brief: they are the reproducibility contract. They are
# deliberately NOT Phase 7's seeds -- a shared seed would make the two gates`
# problem sets identical, and then a problem set that happens to suit one
# planner would flatter both gates at once.
SETS=(
  "panda floor_wall $COUNT 810001"
  "panda cage       $COUNT 810002"
  "fanuc floor_wall $COUNT 810021"
  "fanuc cage       $COUNT 810022"
)
# The oracle`s `plan` op cannot express path constraints, so Phase 7 measures
# this set for condition 2 only. Here it is the same: validity, coverage,
# densification, endpoints and no-timeouts, with no quality check.
CONSTRAINED_SET="panda floor_wall $COUNT 810011 panda_joint1:0.0:0.5"

# The per-planner regression pins, and the only place a constant that decides a
# verdict lives. Every pilot pin below was MEASURED on this tree at
# PILOT_COUNT=8 -- 16 problems per robot, 8 per config, seeds above,
# SEED_BASE=525252, TIMEOUT_SECONDS=120, one process per problem, 125s wall
# clock. PORTING-PLAN.md's Phase 8 property section carries the run and the
# per-set numbers. What each pin was measured at, and why it is where it is:
#
#   chomp endpoint_ceiling 0.0    measured max gap exactly 0.0 over 20 solved
#                                 paths, both robots. Phase 7's exact-zero bar
#                                 transfers to CHOMP unchanged, so it is used
#                                 unchanged -- a ceiling above 0 here would
#                                 accept a regression this tree does not have.
#   stomp endpoint_ceiling 0.03   measured max gap 1.538e-2 (panda, 13 solved)
#                                 and 6.144e-4 (fanuc, 11 solved). NOT near
#                                 zero, and not a defect: see the `endpoints`
#                                 comment below. One value for both robots
#                                 because the mechanism is the smoothing
#                                 filter, not the robot; ~2x the measured
#                                 maximum so 23 solved paths' worth of spread
#                                 does not decide the verdict.
#   cpp_solved_floor              the C++ baseline's own regression bar, two
#                                 below its measurement, the same movement
#                                 allowance `solved_floor` uses. Measured on
#                                 this population at PLANNER_SEED_BASE=525252,
#                                 iteration-bounded (clock 3600s, `timed_out 0`
#                                 on all eight runs): C++ CHOMP panda 12/16
#                                 (floor_wall 6, cage 6) and fanuc 11/16 (5, 6);
#                                 C++ STOMP panda 14/16 (8, 6) and fanuc 12/16
#                                 (6, 6). It watches the BASELINE: a C++ side
#                                 that quietly stopped solving would lower
#                                 `condition1`s bar and raise `condition3`s
#                                 limit together, and both would pass having
#                                 measured less.
#   length_ratio_ceiling 1.05     `max(1.05, 1 + 1.1*(measured-1))` rounded up
#                                 to the nearest 0.05: a 10% allowance on the
#                                 amount by which CHOMP lengthens the path, and
#                                 a 1.05 floor because the measured ratio is at
#                                 1 and a ceiling of exactly 1.00 would be
#                                 decided by the last bit (a direct run of the
#                                 instrument on panda problems 0-3 printed
#                                 1.0000000000000002). Re-measured after the
#                                 metric change below: worst-config ratio
#                                 1.000x on panda (panda_cage, output median
#                                 2.472 / straight-line median 2.471) and
#                                 1.000x on fanuc (fanuc_cage, 1.834 / 1.834).
#
#                                 The 1.013x/1.000x this comment carried before,
#                                 over medians 6.076/5.998 and 6.729/6.729, was
#                                 the same population measured in a different
#                                 metric: both instruments reported Euclidean
#                                 L2 over raw joint values until this round and
#                                 now report `plan_space_length`, the metric the
#                                 C++ baseline`s own `length` is in. The pin is
#                                 unchanged at 1.05 because the rule that sizes
#                                 it -- a 1.05 floor while the measurement sits
#                                 at 1 -- gives the same answer in either.
#
#                                 The 1.433x/1.305x this comment carried before
#                                 was an artifact of the aggregation, not a
#                                 measurement: `merge_rows` took the WORST
#                                 output median across the two configs over the
#                                 BEST seed median across the two configs, so
#                                 the panda number divided panda_cage`s output
#                                 by panda_floor_wall`s seed. `paired_length
#                                 _ratio` is now computed per config, where both
#                                 medians are over the same problems, and only
#                                 the worst *ratio* travels up.
#
#                                 A ratio of one is also exactly what a
#                                 returned-the-seed run looks like -- see
#                                 `chomp/nontrivial-population` -- which is why
#                                 the band alone is not evidence either way.
#   solved_floor                  measured chomp panda 9/16 fanuc 11/16, stomp
#                                 panda 12-13/16 fanuc 11/16; each floor is two
#                                 below the measurement, which is the movement
#                                 Phase 7 (`§248`) measured for the same
#                                 quantity at its own scale. The two on the
#                                 STOMP side is not spare margin: consecutive
#                                 runs of the same seed gave panda_floor_wall
#                                 6 solved / 2 timeouts and then 7 solved / 1
#                                 timeout, one problem sitting on the 120s
#                                 boundary.
#   seed_invalid_floor 1          anti-vacuity, not a rate: at least one solved
#                                 problem in the stratum must have had a
#                                 colliding straight-line seed, by the
#                                 independent checker on the densified seed.
#                                 Measured chomp panda 2 fanuc 0, stomp panda 6
#                                 fanuc 0 -- both fanuc strata FAIL this, and
#                                 correctly: nothing in either fanuc stratum's
#                                 solved set needed an optimizer.
#   mesh_calls_floor 1            anti-vacuity: the injected
#                                 `mesh_to_mesh_collision_free` must be reached
#                                 at least once. Measured 33 (panda) and 17-18
#                                 (fanuc) calls over 16 problems each. A floor
#                                 and not an equality because the count is
#                                 wall-clock dependent: the optimizer calls the
#                                 closure on `iteration % 10 == 0` and leaves
#                                 the loop on `start_time.elapsed() >
#                                 planning_time_limit` (`optimizer.rs:1582`,
#                                 `:1598`), so two runs of the same seed differ
#                                 by an iteration. Two consecutive runs on this
#                                 tree gave fanuc_floor_wall 9 then 10 with
#                                 every other per-set number identical.
#
# `full` is deliberately null. MODE=full has never been run, so there is no
# measured floor for 250 problems per robot, and a floor of 0 would be a check
# that cannot fail dressed as a check that passed. A null stratum raises
# `pins-unmeasured` instead -- see the check list.
PINS_ALL='{
  "full": null,
  "pilot": {
    "chomp": {"panda": {"problems": 16, "solved_floor": 7, "cpp_solved_floor": 10,
                        "endpoint_ceiling": 0.0,
                        "length_ratio_ceiling": 1.05, "mesh_calls_floor": 1,
                        "seed_invalid_floor": 1},
              "fanuc": {"problems": 16, "solved_floor": 9, "cpp_solved_floor": 9,
                        "endpoint_ceiling": 0.0,
                        "length_ratio_ceiling": 1.05, "mesh_calls_floor": 1,
                        "seed_invalid_floor": 1}},
    "stomp": {"panda": {"problems": 16, "solved_floor": 10, "cpp_solved_floor": 12,
                        "endpoint_ceiling": 0.03,
                        "seed_invalid_floor": 1},
              "fanuc": {"problems": 16, "solved_floor": 9, "cpp_solved_floor": 10,
                        "endpoint_ceiling": 0.03,
                        "seed_invalid_floor": 1}}
  }
}'
PINS_JSON="$(jq -c --arg m "$MODE" '.[$m] // {}' <<<"$PINS_ALL")"

# `pinned()` above is keyed `planner.robot` and only reached from the
# `.stratum != null` branch below, so `constrained`/`inject_constrained` --
# which are TAGS, not robots, and have no stratum row at all -- never got a
# `solved_floor`. A full revert of the fix that makes them solvable is caught
# anyway (every `common()` check fails at 0 solved), but erosion is not:
# `> 0` guards pass just as well at 1 solved as at every problem solved, so a
# regression that leaves a few problems solving would pass every row that
# existed before this variable.
#
# This is a SIBLING to $PINS_ALL, not a new case folded into it, because
# $PINS_ALL's shape IS the robot keying -- adding a tag underneath it would
# make one variable answer two different questions ("floor for this robot"
# and "floor for this tag") by context, which is the dual-meaning shape that
# grows a special case per future tag. Keyed `mode -> planner -> tag` instead,
# it answers only the tag question, and reuses `pinned()` unchanged: the
# function only ever read `.problems`/`.solved_floor` off whatever pin object
# it was handed.
#
# `full`'s tags are listed with a null pin, not omitted, unlike $PINS_ALL's
# flatly-null `full`. $PINS_ALL can be flat because `.stratum` rows are
# unconditional -- every planner/robot combination always produces one, so
# `$pins[$p][.robot] == null` alone means "unmeasured" with no ambiguity. A
# `.set` row is not unconditional: most tags (`panda_floor_wall` and so on)
# are never meant to carry a tag-level floor at all, so the check below has to
# tell "this tag carries no floor, ever" apart from "this tag needs a floor
# and none is measured for this mode" -- and a bare `$tag_pins[$p][.tag] //
# null` cannot make that distinction, only `has()` against an entry that is
# actually present can. Listing the tag with a null pin in `full` is what
# lets `has()` see it there too, so `full` fails loud the same way an
# unmeasured stratum does instead of silently skipping the check.
#
# Pinned exactly at the measurement, not two below it the way $PINS_ALL's
# `solved_floor`s are: that margin there is real movement Phase 7 measured
# for STOMP at its own scale, not a blanket rule, and this population showed
# none of it -- three consecutive runs, same tree, same CONSTRAINED_SET tuple
# (panda floor_wall seed 810011 panda_joint1:0.0:0.5) at PILOT_COUNT=8,
# SEED_BASE=525252, TIMEOUT_SECONDS=120, gave the identical count each time
# with zero timeouts on every run: chomp 6/8 solved on both `constrained` and
# `inject_constrained` (identical because CHOMP never plans with
# `joint_constraint` at all -- `optimize_benchmark_chomp.rs:596-608` -- so the
# two tags plan and score identically by construction, not by coincidence),
# stomp 8/8 solved on both (the `7f561e20` fix above). An exact pin on a
# stable measurement has precedent in this same file: `endpoint_ceiling 0.0`
# for CHOMP is pinned at exactly its measurement for the same reason -- "a
# ceiling above 0 here would accept a regression this tree does not have." If
# this population turns out to be noisy on a machine this was never run on,
# that is new evidence for a margin, measured the same way the existing ones
# were -- not a reason to invent one now that nothing here shows.
TAG_PINS_ALL='{
  "full": {"chomp": {"constrained": null, "inject_constrained": null},
           "stomp": {"constrained": null, "inject_constrained": null}},
  "pilot": {
    "chomp": {"constrained": {"problems": 8, "solved_floor": 6},
              "inject_constrained": {"problems": 8, "solved_floor": 6}},
    "stomp": {"constrained": {"problems": 8, "solved_floor": 8},
              "inject_constrained": {"problems": 8, "solved_floor": 8}}
  }
}'
TAG_PINS_JSON="$(jq -c --arg m "$MODE" '.[$m] // {}' <<<"$TAG_PINS_ALL")"

# Which planners this run measures. An override exists for one reason: proving a
# check discriminates means breaking what it watches and showing it fail, and a
# both-planner pilot costs minutes of STOMP wall clock per proof (see the
# `measure-` note at the top). It is honoured in `pilot` mode only -- `full`
# always measures both, so a results file can never record half a measurement
# as a whole one -- and the list is printed on the run banner either way.
if [[ "$MODE" == "full" ]]; then
  PLANNERS="chomp stomp"
else
  PLANNERS="${PLANNERS:-chomp stomp}"
fi
for one in $PLANNERS; do
  case "$one" in
    chomp | stomp) ;;
    *) echo "FAIL unknown planner $one in PLANNERS" >&2; exit 2 ;;
  esac
done

# Capped well under `nproc` so a machine shared with other work is not
# oversubscribed; both instruments are single-threaded per problem (STOMP's
# watchdog thread sleeps), so each shard is one core.
SHARDS="${SHARDS:-16}"

# $WORKDIR was created before this script re-exec'd itself into its own copy
# (see the top of this file) -- not created here, so its cleanup trap is not
# reset here either.

failed=()

# Runs $1 (an instrument binary) over $2's problems, split across $SHARDS
# processes, concatenating the per-problem NDJSON lines into $3. The per-shard
# summary lines are dropped: every aggregate below is recomputed from the
# per-problem lines, which is both what makes sharding transparent and an
# independent check on each binary's own summary arithmetic.
run_sharded() {
  local binary="$1" request="$2" out="$3" timeout="$4" dense="${5:-}"
  local pids=() i rc=0
  # A fresh directory per call, named after the output file: fixed shard names
  # in one shared $WORKDIR let a later, smaller call pick up an earlier call's
  # leftover shard files. `verify-phase7-benchmark.sh` carries the same note
  # because that was a real defect there.
  local dir
  dir="$WORKDIR/shards.$(basename "$out")"
  rm -rf "$dir"
  mkdir -p "$dir"
  : >"$out"
  for ((i = 0; i < SHARDS; i++)); do
    jq --argjson k "$SHARDS" --argjson i "$i" \
      '.problems = [.problems[] | select((.id % $k) == $i)]' \
      "$request" >"$dir/shard.$i.json"
    if [[ "$(jq '.problems|length' "$dir/shard.$i.json")" == "0" ]]; then
      continue
    fi
    "$binary" "$SEED_BASE" "$timeout" "" "$dense" \
      <"$dir/shard.$i.json" >"$dir/shard.$i.ndjson" 2>"$dir/shard.$i.err" &
    pids+=("$!")
  done
  for pid in "${pids[@]}"; do wait "$pid" || rc=1; done
  for ((i = 0; i < SHARDS; i++)); do
    [[ -s "$dir/shard.$i.ndjson" ]] || continue
    jq -c 'select(.id != null)' "$dir/shard.$i.ndjson" >>"$out"
  done

  # Every problem in, every problem out. A shard that died mid-run would
  # otherwise silently shrink the denominator, which reads as a better success
  # rate rather than as a lost shard.
  local want have
  want="$(jq '.problems|length' "$request")"
  have="$(wc -l <"$out")"
  if [[ "$want" != "$have" ]]; then
    echo "FAIL sharded run lost problems: requested $want, got $have ($out)" >&2
    for ((i = 0; i < SHARDS; i++)); do
      [[ -s "$dir/shard.$i.err" ]] && tail -3 "$dir/shard.$i.err" >&2
    done
    rc=1
  fi
  return $rc
}

# The seed every instrument run is given. One value for the whole gate so a
# re-run reproduces exactly; the per-problem stream is `SEED_BASE + problem.id`
# inside each instrument.
SEED_BASE=525252

# Every aggregate this gate needs, recomputed from per-problem lines. `timeout`
# is counted as a failure of its own kind, never folded into `solved` and never
# dropped.
aggregate() {
  jq -s '
    def median: sort | if length==0 then null
      elif (length%2)==1 then .[length/2|floor]
      else (.[length/2-1]+.[length/2])/2 end;
    [.[] | select(.id != null)] |
    (map(select(.solved == true))) as $s |
    {
      problems: length,
      solved: ($s|length),
      timeouts: (map(select(.outcome == "timeout"))|length),
      failures: (map(select(.solved != true and .outcome != "timeout"))|length),
      condition2_checked: ($s|length),
      condition2_pass: ($s|map(select(.condition2_valid == true))|length),
      waypoints_checked: ($s|map(.waypoints_checked // 0)|add // 0),
      raw_waypoints: ($s|map(.raw_waypoints // 0)|add // 0),
      max_endpoint_gap: ($s|map([(.start_gap // 0), (.goal_gap // 0)])|flatten|max // 0),
      slowest_seconds: (map(.plan_seconds // 0)|max // 0),
      # Optimizer-specific. `seed_*`/`output_*` are the paired quantities: the
      # same measure on the same problem, before and after the optimizer, so
      # neither side can improve by failing the hard problems (the survivorship
      # bias `condition3-paired` exists for in Phase 7).
      seed_invalid: ($s|map(select(.seed_valid == false))|length),
      cost_worsened: ($s|map(select((.output_cost // 0) > (.seed_cost // 0)))|length),
      # STOMP only. The gated direction: a seed the independent checker calls
      # colliding must cost something under the planner`s own functional.
      # `cost_fn_margin_only` is the ungated opposite direction (clearance
      # margin), carried so the pass is readable rather than bare.
      cost_fn_missed_seed_collision:
        ($s|map(select(.seed_valid == false and (.seed_cost // 0) <= 0))|length),
      cost_fn_margin_only:
        ($s|map(select(.seed_valid == true and (.seed_cost // 0) > 0))|length),
      cost_fn_seeds_scored: ($s|map(select(.seed_cost != null))|length),
      paired_seed_cost_median: ($s|map(.seed_cost)|map(select(. != null))|median),
      paired_output_cost_median: ($s|map(.output_cost)|map(select(. != null))|median),
      paired_seed_length_median: ($s|map(.seed_length)|map(select(. != null))|median),
      paired_output_length_median: ($s|map(.length)|map(select(. != null))|median),
      # The ratio belongs HERE, at the one level where both medians are taken
      # over the same problems. Computing it after `merge_rows` divides one
      # config`s median by another config`s and yields a number that is no
      # population`s ratio: on this tree that read 1.433x for CHOMP on panda
      # while every solved panda problem`s own ratio was 1.000 or the cage
      # set`s.
      paired_length_ratio:
        (($s|map(.seed_length)|map(select(. != null))|median) as $sm
         | ($s|map(.length)|map(select(. != null))|median) as $om
         | if $sm == null or $om == null or $sm == 0 then null else $om / $sm end),
      mesh_check_calls: (map(.mesh_check_calls // 0)|add // 0),
      mesh_check_true: (map(.mesh_check_true // 0)|add // 0)
    }'
}

# Merges the per-set rows of one robot into that robot's stratum. A stratum is
# a population: the per-robot 500 is one, each config is one, and pooling two
# populations is what lets a failure hide inside a pass -- so this sums the
# counts (which is what a population *is*) and takes the worst of every bound,
# and each config is ALSO gated in its own name below.
merge_rows() {
  jq -s '
    def median: sort | if length==0 then null
      elif (length%2)==1 then .[length/2|floor]
      else (.[length/2-1]+.[length/2])/2 end;
    {
      problems: (map(.problems)|add // 0),
      solved: (map(.solved)|add // 0),
      timeouts: (map(.timeouts)|add // 0),
      failures: (map(.failures)|add // 0),
      condition2_checked: (map(.condition2_checked)|add // 0),
      condition2_pass: (map(.condition2_pass)|add // 0),
      waypoints_checked: (map(.waypoints_checked)|add // 0),
      raw_waypoints: (map(.raw_waypoints)|add // 0),
      max_endpoint_gap: (map(.max_endpoint_gap)|max // 0),
      slowest_seconds: (map(.slowest_seconds)|max // 0),
      seed_invalid: (map(.seed_invalid)|add // 0),
      cost_worsened: (map(.cost_worsened)|add // 0),
      cost_fn_missed_seed_collision: (map(.cost_fn_missed_seed_collision)|add // 0),
      cost_fn_margin_only: (map(.cost_fn_margin_only)|add // 0),
      cost_fn_seeds_scored: (map(.cost_fn_seeds_scored)|add // 0),
      mesh_check_calls: (map(.mesh_check_calls)|add // 0),
      mesh_check_true: (map(.mesh_check_true)|add // 0),
      # Medians of per-set medians are not a median of the pooled set, so these
      # are the *worst* per-set value on each side instead: a pooled median that
      # averaged two sets would be the pooling this gate refuses everywhere
      # else, and the per-set checks below carry the per-set numbers anyway.
      paired_seed_cost_median: (map(.paired_seed_cost_median)|map(select(.!=null))|min),
      paired_output_cost_median: (map(.paired_output_cost_median)|map(select(.!=null))|max),
      paired_seed_length_median: (map(.paired_seed_length_median)|map(select(.!=null))|min),
      paired_output_length_median: (map(.paired_output_length_median)|map(select(.!=null))|max),
      # A ratio may NOT be assembled that way -- worst numerator over one config
      # and best denominator over another is no config`s ratio. The worst
      # *ratio* is, so the whole per-set record travels up and the gate reads
      # this one.
      worst_length_ratio:
        ((map(select(.paired_length_ratio != null))
          | sort_by(.paired_length_ratio) | last)
         | if . == null then null
           else {tag: .tag, ratio: .paired_length_ratio,
                 seed_median: .paired_seed_length_median,
                 output_median: .paired_output_length_median} end)
    }'
}

# Snapshotted HERE, before the build, and not where it is written out at the
# end. The digest has to describe the source this run compiled; taken after the
# measurement it describes whatever is on disk an hour later instead, and an
# edit landing mid-run -- a parallel worker, a fix to the very harness whose
# numbers are being produced -- would be recorded as the code that ran. That
# record then reads as current precisely when it is furthest from true, which
# is the failure `check-measured-sources-current.sh` exists to catch and the
# one shape of it the gate structurally cannot see.
#
# `|| exit 1` on the digest, not `$(...)` inline: this script runs under
# `set -uo pipefail` with no `-e`, so a failed digest inlined into `printf`
# would record an empty value and the record would then disagree with the tree
# for a reason that has nothing to do with drift.
# Same reason, same moment: `working_tree_dirty` and `dirty_paths` describe the
# tree this run measured, and taken at write time they would describe the tree
# an hour of parallel work later. They also carry what the digests structurally
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
# The function comes from the private copy (so its own definition cannot
# version-split mid-run, same as gate-lib.sh above); the directory it hashes
# stays $REPO_ROOT (so the digest describes the real oracle sources, not a
# copy that was never meant to hold them).
# shellcheck source=tools/moveit-oracle/src-digest.sh
source "$(dirname "${BASH_SOURCE[0]}")/../moveit-oracle/src-digest.sh"
ORACLE_STAMP="$(oracle_stamp "$REPO_ROOT/tools/moveit-oracle")" || exit 1

# The list below is the two arms' own algorithm crates plus the validity/cost
# code both arms call to decide what they report -- see the closure argument
# on `measured_source_digest` in gate-lib.sh for how this list was derived
# and what was deliberately left out.
if ! SOURCES_JSON="$(cd "$REPO_ROOT" && for f in \
    tools/ci/measure-phase8-optimizer-properties.sh \
    tools/ci/measure-phase8-cpp-baseline.sh \
    crates/moveit-planners-sbp/examples/plan_benchmark_problem_set.rs \
    crates/moveit-planners-chomp/examples/optimize_benchmark_chomp.rs \
    crates/moveit-planners-stomp/examples/optimize_benchmark_stomp.rs \
    crates/moveit-planners-chomp/src \
    crates/moveit-planners-stomp/src \
    crates/moveit-stomp-core/src \
    crates/moveit-distance-field/src \
    crates/moveit-collision/src \
    crates/moveit-scene/src \
    crates/moveit-constraints/src \
    tools/moveit-oracle/src; do
  d="$(measured_source_digest "$f")" || exit 1
  printf '%s %s\n' "$f" "$d"
done | jq -R -s 'split("\n")|map(select(length>0)|split(" "))|map({key:.[0],value:.[1]})|from_entries')"; then
  echo "FAIL could not digest this run's measured sources -- refusing to start a" >&2
  echo "  measurement whose record could not say what code produced it" >&2
  exit 1
fi

echo "=== building instruments (release) ==="
cargo build --release --manifest-path "$REPO_ROOT/Cargo.toml" \
  -p moveit-planners-sbp -p moveit-planners-chomp -p moveit-planners-stomp \
  --examples || exit 1

# Copied into $WORKDIR the moment the build finishes, and every call below
# uses the copy. Unlike the script/library files above, these are not
# re-exec'd -- a compiled binary is not read incrementally through one fd the
# way bash reads a script, so there is no offset for a rewrite to corrupt
# mid-call, and each call is a fresh, complete `execve` regardless. What a
# copy protects against here is different: `$REPO_ROOT/target/release` is
# shared with every other build in this worktree, and a concurrent `cargo
# build` replacing these paths between two calls this run makes would answer
# some of this run's problems with one version of the instrument and the rest
# with another, while $SOURCES_JSON above named only the one it read at start.
mkdir -p "$WORKDIR/bin" || exit 1
for b in "$GEN" "$CHOMP" "$STOMP"; do
  cp "$b" "$WORKDIR/bin/" || exit 1
done
GEN="$WORKDIR/bin/$(basename "$GEN")"
CHOMP="$WORKDIR/bin/$(basename "$CHOMP")"
STOMP="$WORKDIR/bin/$(basename "$STOMP")"

# --- the discrimination gate ------------------------------------------------
#
# Runs FIRST, and a failure here invalidates every validity number below. A
# checker that silently checks nothing reports 100% exactly as a working one
# does; only a run that must FAIL can tell the two apart. Each instrument
# splices a state, verified bad by direct query before planning starts, into
# every solved path and asserts that none passed -- and asserts first that at
# least one path was checked, because zero checked paths is a vacuous pass.
# Only solved paths can be checked, so the line each instrument prints carries
# the injected count beside the checked one: measured over 125 problems this
# stage checked 85 and 98 for CHOMP and 105 and 119 for STOMP, and a bare
# "rejected all 105 paths" states a numerator as if it were the population.
run_inject() {
  local planner="$1" binary="$2" request="$3" mode="$4" tag="$5"
  if "$binary" "$SEED_BASE" "$TIMEOUT_SECONDS" "$mode" \
       <"$request" >"$WORKDIR/inject.$planner.$tag.json" \
       2>"$WORKDIR/inject.$planner.$tag.err"; then
    echo "  PASS inject=$mode $planner/$tag -- $(tail -1 "$WORKDIR/inject.$planner.$tag.err")"
  else
    echo "  FAIL inject=$mode $planner/$tag did not reject every injected path:" >&2
    tail -6 "$WORKDIR/inject.$planner.$tag.err" >&2
    failed+=("inject=$mode $planner/$tag")
  fi
}

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
echo "=== validity discrimination gate (injected bad waypoints must be rejected) ==="
for planner in $PLANNERS; do
  case "$planner" in
    chomp) binary="$CHOMP" ;;
    stomp) binary="$STOMP" ;;
  esac
  # `collision` needs obstacles for the spliced state to hit; `constraint` needs
  # a constraint for it to violate.
  [[ -s "$WORKDIR/panda_floor_wall.request.json" ]] || continue
  run_inject "$planner" "$binary" "$WORKDIR/panda_floor_wall.request.json" \
             collision panda_floor_wall
  # `constraint` USED TO be unable to use the constrained set as generated:
  # STOMP solved none of it -- measured 0 solved / 16 timeouts over three
  # constrained sets (seed 810011 floor_wall tolerance 0.5, 8 problems; seed
  # 810012 floor_wall tolerance 1.5, 4; seed 810013 cage tolerance 0.5, 4), and
  # one of those problems given 600s instead of 120s was still not solved when
  # it was killed at 730s -- because `getConstraintsCostFunction` costs a state
  # by `decide().distance`, the distance to the constraint TARGET rather than
  # the amount by which the tolerance is exceeded, and
  # `cost_function_from_state_validator` marked any timestep with cost > 0
  # invalid. A group that moved the constrained joint at all therefore had no
  # valid timestep, at any budget. With no solved path there was nothing to
  # splice a bad waypoint into, and the stage reported "checked nothing".
  #
  # That premise is now false: `7f561e20` (fix(stomp): gate constraint cost on
  # satisfied, not raw distance) removed the cause. Measured on this exact
  # `CONSTRAINED_SET` tuple (panda floor_wall seed 810011
  # panda_joint1:0.0:0.5), 16 problems, PLANNER_SEED_BASE 525252, control
  # `ff045455` vs treatment `ff045455`+`7f561e20` cherry-picked: solved 0/16 ->
  # 16/16, timeouts 16/16 -> 0/16, condition2_pass 0/16 -> 16/16, median
  # plan_seconds 120.07 -> 3.98. Full design and caveats (this is 16/16 at
  # PILOT_COUNT, not a `full`-mode measurement) in
  # `scratchpad/stompmeas/RESULT.md` (2026-08-08, run in a throwaway
  # integration worktree, not this tree).
  #
  # So `constrained.request.json` as generated is now a real population to
  # inject against, and gets its own arm below using the SAME constraint field
  # for both planning and checking -- the common-case configuration, unmeasured
  # by this gate until now because there was nothing solved to measure it on.
  #
  # That does not retire the split-field arm beneath it. `check_joint_constraint`
  # exists as its own field precisely for "check against a constraint the
  # planner did not necessarily see" -- a distinct, real configuration this
  # binary supports -- so `inject_constrained` (constraint moved from
  # `joint_constraint` to `check_joint_constraint`, planned WITHOUT it and
  # checked WITH it) keeps running alongside `constrained`, not in place of it.
  # `<planner>/inject_constrained` in the run list below is that exact request
  # without the injection, and it has to come out 100% valid for the rejection
  # here to mean the waypoint was rejected rather than the path.
  [[ -s "$WORKDIR/constrained.request.json" ]] || continue
  # The single-field arm: `joint_constraint` alone, unmodified, driving both
  # planning and (via `check_joint_constraint`'s `or_else` fallback in
  # `optimize_benchmark_stomp.rs`) checking.
  run_inject "$planner" "$binary" "$WORKDIR/constrained.request.json" \
             constraint constrained
  jq '(.check_joint_constraint = .joint_constraint) | .joint_constraint = null' \
    "$WORKDIR/constrained.request.json" >"$WORKDIR/inject_constrained.request.json"
  run_inject "$planner" "$binary" "$WORKDIR/inject_constrained.request.json" \
             constraint inject_constrained
done

echo
echo "=== instrument runs, planners=[$PLANNERS], timeout=${TIMEOUT_SECONDS}s per call, ${SHARDS} shards ==="
run_start=$(date +%s.%N)
for planner in $PLANNERS; do
  case "$planner" in
    chomp) binary="$CHOMP" ;;
    stomp) binary="$STOMP" ;;
  esac
  # `inject_constrained` is the constrained set with its constraint moved from
  # the planner to the checker (written by the injection stage above). It is run
  # here WITHOUT injection because that is the other half of the injection
  # stage's pair: these paths must come out 100% valid under the same checker
  # constraint the injected run is rejected under, or the rejection there says
  # nothing about the spliced waypoint.
  for entry in "${SETS[@]}" "constrained x x x" "inject_constrained x x x"; do
    read -r robot config count seed <<<"$entry"
    case "$robot" in
      constrained | inject_constrained) tag="$robot" ;;
      *) tag="${robot}_${config}" ;;
    esac
    [[ -s "$WORKDIR/$tag.request.json" ]] || continue
    if ! run_sharded "$binary" "$WORKDIR/$tag.request.json" \
                     "$WORKDIR/$planner.$tag.ndjson" "$TIMEOUT_SECONDS" dense; then
      failed+=("$planner run $tag")
      continue
    fi
    aggregate <"$WORKDIR/$planner.$tag.ndjson" >"$WORKDIR/$planner.$tag.agg.json"
    echo "  $planner/$tag: $(jq -c '{problems, solved, timeouts, failures, condition2_pass, seed_invalid, mesh_check_calls}' "$WORKDIR/$planner.$tag.agg.json")"
  done
done
run_seconds=$(echo "$(date +%s.%N) - $run_start" | bc)

# --- the C++ baseline -------------------------------------------------------
#
# Upstream's own CHOMP and STOMP over the SAME request files this port just
# consumed -- `measure-phase8-cpp-baseline.sh` is given `SET_FILE`, so the two
# sides see the same bytes rather than two generator runs argued to agree.
#
# One oracle process per problem at `--planner-rng-seed $((SEED_BASE + id))`,
# because upstream draws from `rsl::rng()`, a `thread_local std::mt19937`
# seedable exactly once per thread; that is also the per-problem stream each
# port instrument uses, so the comparison is problem by problem and not just
# rate against rate.
#
# The clock bounds are left at that script's defaults (3600s) so the ITERATION
# bound terminates. A C++ rate measured at upstream's own wall-clock defaults
# would be a measurement of this machine, and the baseline script hard-fails on
# any `TIMED_OUT` rather than reporting such a rate.
#
# The four stratum sets only. `chomp_plan` and `stomp_plan` never read the
# request's `joint_constraint` (`oracle.cpp`: it is read at `:562` and `:5166`,
# by other ops), so a C++ run over the `constrained` set would plan a different
# problem than the port did -- exactly the substitution a baseline exists to
# prevent. Phase 7 treats its constrained set as condition-2-only for the
# analogous reason, and so does this file.
CPP_BASELINE="$REPO_ROOT/tools/ci/measure-phase8-cpp-baseline.sh"
echo
echo "=== C++ baseline: upstream CHOMP/STOMP over the same sets, one process per problem ==="
cpp_start=$(date +%s.%N)
for planner in $PLANNERS; do
  for entry in "${SETS[@]}"; do
    read -r robot config count seed <<<"$entry"
    tag="${robot}_${config}"
    [[ -s "$WORKDIR/$tag.request.json" ]] || continue
    dir="$WORKDIR/cpp.$planner.$tag"
    if ! SET_FILE="$WORKDIR/$tag.request.json" PLANNER_SEED_BASE="$SEED_BASE" \
         CHOMP_CLOCK_BOUND="$CPP_CLOCK_BOUND" STOMP_CLOCK_BOUND="$CPP_CLOCK_BOUND" \
         JOBS="$SHARDS" "$CPP_BASELINE" "$planner" "$config" "$count" "$seed" "$dir" \
         >"$WORKDIR/cpp.$planner.$tag.log" 2>&1; then
      echo "  FAIL C++ baseline $planner/$tag:" >&2
      tail -5 "$WORKDIR/cpp.$planner.$tag.log" >&2
      failed+=("cpp baseline $planner/$tag")
      continue
    fi
    cp "$dir/$planner.$config.ndjson" "$WORKDIR/$planner.$tag.cpp.ndjson"
    echo "  $(tail -1 "$WORKDIR/cpp.$planner.$tag.log")"
  done
done
cpp_seconds=$(echo "$(date +%s.%N) - $cpp_start" | bc)

# The two sides' rates, medians, and the medians over the problems BOTH solved.
#
# Keyed `tag#id`, never by bare id: every set numbers its problems from 0, so
# pooling two sets by id alone would pair `floor_wall` problem 7 with `cage`
# problem 7. `verify-phase7-benchmark.sh`'s `pooled_medians` carries the same
# note, and this is its counterpart.
#
# `length` on both sides is now the SAME metric -- `JointModelGroupSpace::
# distance` summed along the path on this side, `planSpacePathLength` summing
# OMPL `CompoundStateSpace::distance` on the oracle's. Until this round the
# port instruments reported Euclidean L2 over raw joint values, a different
# quantity, and no ratio between the two sides meant anything.
#
# $1 planner, $2 a name for the pooled files, then its tags. Emits one JSON
# object, or `null` if no tag had both sides.
cpp_medians() {
  local planner="$1" label="$2"
  shift 2
  local pc="$WORKDIR/pooled.$planner.$label.port.ndjson"
  local cc="$WORKDIR/pooled.$planner.$label.cpp.ndjson"
  : >"$pc"
  : >"$cc"
  local tag any=0
  for tag in "$@"; do
    [[ -s "$WORKDIR/$planner.$tag.ndjson" && -s "$WORKDIR/$planner.$tag.cpp.ndjson" ]] || continue
    any=1
    jq -c --arg tag "$tag" \
      'select(.id != null) | {key: ($tag + "#" + (.id|tostring)), solved: (.solved == true), length}' \
      "$WORKDIR/$planner.$tag.ndjson" >>"$pc"
    # `wall` is carried from the C++ side only, and only to answer one
    # question: whether the clock the C++ arm was given is what stopped any of
    # its calls. `stratum.slowest_seconds` already records the same for the
    # port arm, so with `clock_bounds` above the record finally holds all four
    # numbers a reader needs to tell a budget artifact from a port defect.
    # Before this it held the port arm's bound and slowest call and neither of
    # the C++ arm's, which is how a 6.0s-vs-3600s split published a FAIL.
    jq -c --arg tag "$tag" \
      'select(.id != null) | {key: ($tag + "#" + (.id|tostring)), solved: (.solved == true), length,
                              wall: (.wall_secs? // null)}' \
      "$WORKDIR/$planner.$tag.cpp.ndjson" >>"$cc"
  done
  if [[ "$any" == "0" ]]; then
    echo 'null'
    return 0
  fi
  jq -n --slurpfile p <(jq -s '.' "$pc") --slurpfile c <(jq -s '.' "$cc") '
    def median: map(select(. != null)) | sort
      | if length==0 then null
        elif (length%2)==1 then .[length/2|floor]
        else (.[length/2-1]+.[length/2])/2 end;
    ($p[0]) as $pt | ($c[0]) as $cp |
    ([$pt[]|select(.solved)|.key]) as $pk |
    ([$cp[]|select(.solved)|.key]) as $ck |
    (($ck - ($ck - $pk))|unique) as $both |
    {
      cpp_problems: ($cp|length),
      cpp_solved: ($ck|length),
      cpp_rate: (if ($cp|length) > 0 then ($ck|length)/($cp|length) else 0 end),
      port_rate: (if ($pt|length) > 0 then ($pk|length)/($pt|length) else 0 end),
      cpp_median_length: ([$cp[]|select(.solved)|.length]|median),
      port_median_length: ([$pt[]|select(.solved)|.length]|median),
      paired_problems: ($both|length),
      cpp_paired_median: ([$cp[]|select(.solved and (.key as $k|$both|index($k)))|.length]|median),
      port_paired_median: ([$pt[]|select(.solved and (.key as $k|$both|index($k)))|.length]|median),
      cpp_slowest_seconds: ([$cp[]|.wall|select(. != null)]|if length==0 then null else max end),
      cpp_wall_secs_recorded: ([$cp[]|.wall|select(. != null)]|length)
    }'
}

# --- the independent cross-check -------------------------------------------
#
# `is_path_valid` (what the validity check reports) and the collision cost the
# optimizer minimized are two entry points to one `ParryCollisionEnv`, and the
# injected state was found by asking that same env through a third. So the
# injection gate proves the entry points agree with each other -- not that the
# collision model is right. A backend that misses a contact produces a
# colliding path AND approves it, and every check that only ever asks this port
# stays green. Only a different implementation can see that.
#
# The oracle's `is_state_valid` op is one: upstream MoveIt's
# `PlanningScene::isPathValid` over FCL, whose loop is per-waypoint with no
# interpolation of its own (`moveit_core/planning_scene/src/planning_scene.cpp
# :2365-2424`, at the pinned sha), so handing it the same densified waypoint
# list this port just checked asks two implementations the same question about
# the same states. That per-waypoint shape is also why many paths share one
# invocation, and why `goal_constraints` is left empty.
#
# $1 label, $2 robot, $3 the request the paths came from, $4 NDJSON with
# `dense`, $5 expected verdict, $6 joint-constraint spec or "".
oracle_path_check() {
  local label="$1" robot="$2" request="$3" ndjson="$4" expect="$5" spec="$6"
  local isv="$WORKDIR/isv.$label.ndjson" out="$WORKDIR/isv.$label.out"
  local pc='{}'
  if [[ -n "$spec" ]]; then
    pc="$(jq -c -n --arg spec "$spec" '($spec|split(":")) as $p |
      {joint_constraints:[{joint_name:$p[0], position:($p[1]|tonumber),
        tolerance_above:($p[2]|tonumber), tolerance_below:($p[2]|tonumber),
        weight:1.0}]}')"
  fi
  jq -c --slurpfile req "$request" --argjson pc "$pc" \
    'select(.dense != null)
     | {id, op:"is_state_valid", objects: $req[0].objects,
        path_constraints: $pc, waypoints: .dense}' "$ndjson" >"$isv"

  local n_paths
  n_paths="$(wc -l <"$isv")"
  if [[ "$n_paths" -eq 0 ]]; then
    echo "  FAIL cross-check $label: no path carried \`dense\`, nothing was compared" >&2
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
        same_indices: ((.result.invalid_waypoints // []) == ($pp.invalid_waypoints // [])),
        n_oracle: ((.result.invalid_waypoints // [])|length),
        n_port: (($pp.invalid_waypoints // [])|length) } ] as $rows |
    { paths: ($rows|length),
      not_ok: [$rows[]|select(.ok|not)|.id],
      wrong_verdict: [$rows[]|select(.oracle_valid != ($expect == "valid"))|.id],
      disagree: [$rows[]|select(.oracle_valid != .port_valid)|.id],
      index_mismatch: [$rows[]|select(.same_indices|not)|{id, n_oracle, n_port}] }')"
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

echo
echo "=== cross-check: every produced path through upstream isPathValid ==="
for planner in $PLANNERS; do
  for entry in "${SETS[@]}"; do
    read -r robot config count seed <<<"$entry"
    tag="${robot}_${config}"
    [[ -s "$WORKDIR/$planner.$tag.ndjson" ]] || continue
    oracle_path_check "$planner.$tag" "$robot" "$WORKDIR/$tag.request.json" \
      "$WORKDIR/$planner.$tag.ndjson" valid ""
  done
  # Both constrained populations, each against the constraint it was CHECKED
  # with: `constrained` planned with it, `inject_constrained` only checked with
  # it. Upstream gets the same spec either way -- the oracle never plans here,
  # it re-checks this port's own waypoints.
  for tag in constrained inject_constrained; do
    [[ -s "$WORKDIR/$planner.$tag.ndjson" ]] || continue
    oracle_path_check "$planner.$tag" "$c_robot" \
      "$WORKDIR/$tag.request.json" \
      "$WORKDIR/$planner.$tag.ndjson" valid "$c_spec"
  done
done

# --- the verdict ------------------------------------------------------------
#
# One decision list, printed and gated in a single pass, so the printed verdict
# and the exit code cannot disagree. Building it as data (rather than printing
# and then re-deriving the exit code) is what `verify-phase7-benchmark.sh`
# `§248` had to fix there: a stratum printed FAIL while the exit code stayed 0.
verdict_json="$WORKDIR/verdict.json"
: >"$verdict_json"
for planner in $PLANNERS; do
  for robot in panda fanuc; do
    rows=()
    for config in floor_wall cage; do
      f="$WORKDIR/$planner.${robot}_${config}.agg.json"
      [[ -s "$f" ]] || continue
      # Tagged before merging so `worst_length_ratio` can name the config whose
      # ratio decided the verdict.
      jq -c --arg t "${robot}_${config}" '. + {tag: $t}' "$f" >"$f.tagged"
      rows+=("$f.tagged")
    done
    [[ ${#rows[@]} -gt 0 ]] || continue
    # Pooled over this stratum's sets, from the per-problem lines of both
    # sides, so `condition3-pooled` is a median of the pooled population and
    # not a median of two per-set medians.
    cpp="$(cpp_medians "$planner" "$robot" "${robot}_floor_wall" "${robot}_cage")"
    cat "${rows[@]}" | merge_rows \
      | jq -c --arg p "$planner" --arg r "$robot" --argjson cpp "$cpp" \
        '{planner:$p, robot:$r, stratum:., cpp:$cpp}' \
      >>"$verdict_json"
  done
  for entry in "${SETS[@]}"; do
    read -r robot config count seed <<<"$entry"
    f="$WORKDIR/$planner.${robot}_${config}.agg.json"
    [[ -s "$f" ]] || continue
    cpp="$(cpp_medians "$planner" "${robot}_${config}" "${robot}_${config}")"
    jq -c --arg p "$planner" --arg t "${robot}_${config}" --argjson cpp "$cpp" \
      '{planner:$p, tag:$t, set:., cpp:$cpp}' "$f" >>"$verdict_json"
  done
  for tag in constrained inject_constrained; do
    f="$WORKDIR/$planner.$tag.agg.json"
    [[ -s "$f" ]] || continue
    jq -c --arg p "$planner" --arg t "$tag" '{planner:$p, tag:$t, set:.}' "$f" >>"$verdict_json"
  done
done

checks_json="$WORKDIR/checks.json"
jq -s -c --argjson pins "$PINS_JSON" --argjson tag_pins "$TAG_PINS_JSON" \
     --argjson tmo "$TIMEOUT_SECONDS" \
     --argjson cppclock "$CPP_CLOCK_BOUND" \
     --arg mode "$MODE" '
  def r3($x): if $x == null then null else (($x)*1000|round)/1000 end;
  def pct($x): (($x//0)*10000|round)/100;
  def ratio($n; $d): (if ($d//0) > 0 then ($n//0)/$d else null end);

  # Condition 1 compares two success RATES, and that is a statement about the
  # port only when both arms had the same chance to finish. The two arms run
  # at different clocks by design ($tmo and $cppclock), which is harmless while
  # neither binds -- and stops being harmless the moment the port arm records a
  # timeout, because then its rate is partly a property of its budget.
  #
  # The verdict is deliberately NOT changed, in either direction. A timeout can
  # only lower the port rate, so a PASS carrying timeouts is conservative and
  # needs no caveat; a FAIL carrying them is the ambiguous one, and what is
  # wrong there is not the boolean but reading the number as a quality finding.
  # This note is what stops that reading. It exists because I made exactly that
  # mistake from this files own output: STOMP 185/250 against C++ 217/250 was
  # relayed as "the port is too slow" when every one of the 65 missing problems
  # was a timeout and the C++ arm had 30x the budget.
  def budget_note($s; $c):
    if ($s.timeouts//0) == 0 then ""
    else " -- NOT a clean rate comparison: the port arm timed out on "
         + "\($s.timeouts) of \($s.problems) calls at its \($tmo)s bound while "
         + "the C++ arm ran at \($cppclock)s and its slowest call was "
         + (if ($c.cpp_slowest_seconds // null) == null then "not recorded"
            else "\(r3($c.cpp_slowest_seconds))s" end)
         + ", so the port rate here is a lower bound"
    end;

  # Every check that transfers from Phase 7 unchanged, plus the two the
  # optimizer shape replaces. `$label` is "<planner>/<population>" so no two
  # populations can ever share a check name -- the per-stratum rule, enforced
  # by construction rather than by convention.
  def common($s; $label):
    [
      { name: "\($label)/validity",
        detail: "\($s.condition2_pass)/\($s.condition2_checked) paths valid, \($s.waypoints_checked) waypoints checked",
        ok: (($s.condition2_checked//0) > 0 and $s.condition2_pass == $s.condition2_checked) },
      # A solved path whose validity was never checked is invisible to the
      # validity check: `condition2_checked > 0` is satisfied by one path out
      # of five hundred.
      { name: "\($label)/validity-covers-every-solved-path",
        detail: "checked \($s.condition2_checked) of \($s.solved) solved",
        ok: (($s.solved//0) > 0 and $s.condition2_checked == $s.solved) },
      # Densification is what makes the validity check independent of the
      # waypoints the optimizer already scored, so a collapse back to the
      # returned points has to fail here, not report 100%. It matters more here
      # than in Phase 7: both instruments return a fixed, coarse point count
      # (STOMP `num_timesteps`, CHOMP `3.0/0.03`), so the joint motion between
      # two returned points is unbounded by anything the planner checked.
      { name: "\($label)/validity-densified",
        detail: "\($s.waypoints_checked) checked from \($s.raw_waypoints) returned",
        ok: (($s.raw_waypoints//0) > 0 and ($s.waypoints_checked//0) > ($s.raw_waypoints//0)) },
      # A run containing a timeout is not reproducible -- the same tree on a
      # slower machine gives a different answer.
      { name: "\($label)/no-timeouts",
        detail: "\($s.timeouts) timeouts, slowest call \(r3($s.slowest_seconds))s of the \($tmo)s budget",
        ok: (($s.timeouts//1) == 0) }
    ];

  # `solved` says a trajectory came back, not that it runs between the two
  # states asked for. Unlike Phase 7 this is a pinned ceiling rather than exact
  # zero, and for STOMP the ceiling is not near zero either:
  # `filter_functions::simple_smoothing_matrix` right-multiplies the WHOLE
  # trajectory by `generate_smoothing_matrix(num_timesteps, 1.0)`, which is the
  # normalised inverse of the control-cost matrix R -- not a matrix whose first
  # and last rows are identity -- so timestep 0 and timestep N-1 are smoothed
  # along with every interior point, exactly as upstream`s `simpleSmoothingMatrix`
  # does (`filter_functions.hpp` lines 71-77, `row.transpose() = smoothing_matrix
  # * row.transpose()` over every row). The measured drift is in the plan
  # section. CHOMP is the other case: it pins index 0 and the goal index, and its
  # measured gap is exact 0, so its ceiling is near zero and would catch a
  # regression.
  def endpoints($s; $label; $pin):
    { name: "\($label)/endpoints",
      detail: "max gap from requested start/goal over solved paths: \($s.max_endpoint_gap), ceiling \($pin.endpoint_ceiling)",
      ok: (($s.max_endpoint_gap//1) <= $pin.endpoint_ceiling) };

  def pinned($s; $label; $pin):
    [
      { name: "\($label)/pin-population",
        detail: "\($s.problems) problems, pinned at \($pin.problems)",
        ok: (($s.problems//0) == $pin.problems) },
      { name: "\($label)/no-regression-solved",
        detail: "solved \($s.solved)/\($s.problems) >= floor \($pin.solved_floor)",
        ok: (($s.solved//0) >= $pin.solved_floor) }
    ];

  # Phase 7`s cpp-baseline floor, transferred. It watches the BASELINE, not the
  # port: a C++ side that quietly stopped solving would lower `condition1`s bar
  # and raise `condition3`s limit at the same time, so both would pass while
  # measuring less. That is the whole reason the check exists there.
  def pinned_cpp($c; $label; $pin):
    [
      { name: "\($label)/no-regression-cpp-solved",
        detail: "C++ solved \($c.cpp_solved)/\($c.cpp_problems) >= floor \($pin.cpp_solved_floor)",
        ok: (($c.cpp_solved//0) >= $pin.cpp_solved_floor) }
    ];

  # Phase 7`s conditions 1 and 3, restored -- see the header. The baseline is
  # this planner`s OWN C++ implementation over the same problems at the same
  # per-problem seed, not C++ OMPL RRTConnect, which is what makes the bar a
  # statement about the port rather than about algorithm class.
  def cpp_stratum($s; $c; $label):
    [
      { name: "\($label)/condition1",
        detail: "port \($s.solved)/\($s.problems) = \(pct($c.port_rate))% >= 0.9 x C++ \($c.cpp_solved)/\($c.cpp_problems) = \(pct(($c.cpp_rate//0)*0.9))%\(budget_note($s; $c))",
        # `cpp_problems > 0` is not decoration: without it an empty baseline
        # reads 0 >= 0 and passes, which is how a stage that measured nothing
        # reports success.
        ok: (($c.cpp_problems//0) > 0 and ($c.port_rate//0) >= (($c.cpp_rate//0)*0.9)) },
      { name: "\($label)/condition3-pooled",
        detail: "port median \(r3($c.port_median_length)) vs limit \(r3(($c.cpp_median_length//0)*1.3)) (ratio \(r3(ratio($c.port_median_length; $c.cpp_median_length)))x)",
        ok: (($c.cpp_median_length//0) > 0 and ($c.port_median_length//0) <= ($c.cpp_median_length*1.3)) },
      # The same 1.3x over the problems BOTH sides solved. Condition 3 as
      # written takes each side`s median over its own solved set, and a port
      # that fails the hard problems drops their long paths out of its own
      # median -- passing more easily the worse it gets.
      { name: "\($label)/condition3-paired",
        detail: "over \($c.paired_problems) problems both sides solved: port \(r3($c.port_paired_median)) vs limit \(r3(($c.cpp_paired_median//0)*1.3)) (ratio \(r3(ratio($c.port_paired_median; $c.cpp_paired_median)))x)",
        ok: (($c.paired_problems//0) > 0 and ($c.cpp_paired_median//0) > 0
             and ($c.port_paired_median//0) <= ($c.cpp_paired_median*1.3)) }
    ];

  # The same two conditions at set granularity. Pooling `floor_wall` with
  # `cage` averages two populations just as much as averaging panda with fanuc
  # does, so each set is gated in its own name too.
  def cpp_set($s; $c; $label):
    [
      { name: "\($label)/condition1",
        detail: "port \($s.solved)/\($s.problems) = \(pct($c.port_rate))% >= 0.9 x C++ \($c.cpp_solved)/\($c.cpp_problems) = \(pct(($c.cpp_rate//0)*0.9))%\(budget_note($s; $c))",
        ok: (($c.cpp_problems//0) > 0 and ($c.port_rate//0) >= (($c.cpp_rate//0)*0.9)) },
      { name: "\($label)/condition3",
        detail: "port median \(r3($c.port_median_length)) vs limit \(r3(($c.cpp_median_length//0)*1.3)) (ratio \(r3(ratio($c.port_median_length; $c.cpp_median_length)))x)",
        ok: (($c.cpp_median_length//0) > 0 and ($c.port_median_length//0) <= ($c.cpp_median_length*1.3)) }
    ];

  # CHOMP: its own objective (smoothness + obstacle cost) is not returned by
  # `solve`, so a paired improvement check cannot be built against the existing
  # entry point -- see the plan section. What is observable is the returned
  # path length against the straight line between the same two endpoints, the
  # shortest any trajectory between them can be. A ratio band is a regression
  # bar, not a quality claim, and it is labelled as one.
  def chomp_quality($s; $label; $pin):
    [
      { name: "\($label)/length-ratio-band",
        detail: (if $s.worst_length_ratio == null
                 then "no solved problem carried a length, nothing was compared"
                 else "worst config \($s.worst_length_ratio.tag): output median \(r3($s.worst_length_ratio.output_median)) / straight-line median \(r3($s.worst_length_ratio.seed_median)) = \(r3($s.worst_length_ratio.ratio))x, ceiling \($pin.length_ratio_ceiling)x" end),
        ok: ($s.worst_length_ratio != null
             and $s.worst_length_ratio.ratio <= $pin.length_ratio_ceiling) },
      # The `mesh_to_mesh_collision_free` closure this instrument supplies is
      # upstream`s `isCurrentTrajectoryMeshToMeshCollisionFree`. Every other
      # caller in the tree passes `|_, _| false`, so if it is never called the
      # wiring is vouching for a path upstream has and this run does not.
      { name: "\($label)/mesh-check-exercised",
        detail: "\($s.mesh_check_true)/\($s.mesh_check_calls) mesh-to-mesh checks returned collision-free",
        ok: (($s.mesh_check_calls//0) >= $pin.mesh_calls_floor) },
      # The same anti-vacuity rule as STOMP`s, and for CHOMP it is what makes
      # `length-ratio-band` mean anything: if the straight line between the two
      # endpoints was already collision-free, CHOMP breaks out on its first
      # iteration and the "output" IS the seed. Then validity is 100%, the
      # endpoints are exact, and the length ratio is 1.000 -- every number above
      # passes while measuring the problem generator instead of the optimizer.
      { name: "\($label)/nontrivial-population",
        detail: "\($s.seed_invalid) of \($s.solved) solved problems had a colliding straight-line seed, floor \($pin.seed_invalid_floor)",
        ok: (($s.seed_invalid//0) >= $pin.seed_invalid_floor) }
    ];

  # STOMP. There is deliberately NO `output_cost <= seed_cost` check here.
  # `Stomp::solve` returns `parameters_valid` (`stomp.rs:601`) and
  # `cost_function_from_state_validator` clears that flag for any column with
  # `costs(t) > 0.0` (`cost_functions.rs:174,199`), where every column is a sum
  # of non-negative validator penalties. So `solved == true` forces
  # `output_cost == 0` and `seed_cost >= 0` forces the comparison to hold: the
  # check could not fail on any run this gate can produce. It was in this gate
  # and it passed with "output cost median 0 <= seed cost median 0" on both
  # robots, having measured nothing. Measured directly, panda floor_wall
  # problems 0-3 at SEED_BASE=525252: output_cost 0.0/0.0/0.0 with seed_cost
  # 0.0/19.0/0.0.
  #
  # What survives is the direction the solver does NOT determine: the ported
  # distance-field cost must see the collisions the independent checker sees.
  def stomp_quality($s; $label; $pin):
    [
      { name: "\($label)/cost-fn-sees-seed-collisions",
        detail: "\($s.cost_fn_missed_seed_collision) of \($s.seed_invalid) checker-invalid seeds scored cost 0 (must be 0); \($s.cost_fn_margin_only) of \($s.cost_fn_seeds_scored) scored >0 while the checker passed them (clearance margin, ungated)",
        ok: (($s.cost_fn_missed_seed_collision//1) == 0) },
      # A population whose seeds were all already valid measures nothing about
      # the optimizer: STOMP returns the straight line and every check passes.
      # This is the same anti-vacuity rule as `validity-densified`, applied to
      # the problem set instead of the waypoints. It is also what gives
      # `cost-fn-sees-seed-collisions` a non-empty population to check.
      { name: "\($label)/nontrivial-population",
        detail: "\($s.seed_invalid) of \($s.solved) solved problems had a colliding straight-line seed, floor \($pin.seed_invalid_floor)",
        ok: (($s.seed_invalid//0) >= $pin.seed_invalid_floor) }
    ];

  def quality($planner; $s; $label; $pin):
    if $planner == "chomp" then chomp_quality($s; $label; $pin)
    else stomp_quality($s; $label; $pin) end;

  [ .[] | select(.stratum != null)
    | .planner as $p | "\(.planner)/\(.robot)" as $label
    | (($pins[$p][.robot]) // null) as $pin
    | .stratum as $s
    | .cpp as $c
    | common($s; $label)
      + (if $c == null then
           # Same rule as `pins-unmeasured`: a stratum whose C++ baseline did
           # not run loses condition1, both condition3 rows and
           # no-regression-cpp-solved. Dropping them silently would report a
           # four-check-lighter stratum as a checked one.
           [{ name: "\($label)/cpp-baseline-missing",
              detail: "no C++ baseline for \($label), so condition1, condition3-pooled, condition3-paired and no-regression-cpp-solved did not run",
              ok: false }]
         else cpp_stratum($s; $c; $label)
              + (if $pin == null then [] else pinned_cpp($c; $label; $pin) end) end)
      + (if $pin == null then
           # A mode with no measured pins loses `endpoints`, both `pinned`
           # checks and both quality checks -- six of this stratum`s ten. An
           # empty list here would print the four that remain and reach the exit
           # clean, i.e. report a partially-checked run as a checked one. One
           # named failure instead, carrying what is missing.
           [{ name: "\($label)/pins-unmeasured",
              detail: "mode=\($mode) has no measured pins for \($p)/\(.robot), so endpoints, pin-population, no-regression-solved, no-regression-cpp-solved and both quality checks did not run",
              ok: false }]
         else
           [endpoints($s; $label; $pin)] + pinned($s; $label; $pin)
           + quality($p; $s; $label; $pin) end) ]
  + [ .[] | select(.set != null)
      | .planner as $p | .tag as $t | "\($p)/\($t)" as $label | .set as $s | .cpp as $c
      | common($s; $label)
        # The two constrained populations carry no `cpp` at all, by the
        # decision in the C++ baseline stage above: `chomp_plan`/`stomp_plan`
        # never read `joint_constraint`, so a C++ run over them would answer a
        # different question. Their absence is a property of the population,
        # not of a stage that failed, which is why it is not a failure row the
        # way `cpp-baseline-missing` is on a stratum.
        + (if $c == null then [] else cpp_set($s; $c; $label) end)
        # `has()` membership, not `// null` coalescing: most tags here
        # (`panda_floor_wall` and so on) are never meant to carry a
        # tag-level floor at all -- they are already pinned via their own
        # `.stratum` row above -- so they must fall through this untouched,
        # not gain a spurious new check. Only a tag `$tag_pins[.planner]`
        # actually lists (`constrained`, `inject_constrained`) is a member
        # of the tag-pinned population; `full`s tags are listed with a null
        # pin for exactly this reason -- see `$TAG_PINS_ALL`.
        + (($tag_pins[$p] // {}) as $tp
           | if ($tp | has($t)) then
               ($tp[$t]) as $pin
               | if $pin == null then
                   [{ name: "\($label)/pins-unmeasured",
                      detail: "mode=\($mode) has no measured tag pin for \($label), so pin-population and no-regression-solved did not run",
                      ok: false }]
                 else pinned($s; $label; $pin) end
             else [] end) ]
  | flatten
' "$verdict_json" >"$checks_json"

echo
echo "=== Phase 8 optimizer properties ==="
while IFS=$'\t' read -r name ok detail; do
  if [[ "$ok" == "true" ]]; then
    printf '  PASS %-56s %s\n' "$name" "$detail"
  else
    printf '  FAIL %-56s %s\n' "$name" "$detail" >&2
    failed+=("$name")
  fi
done < <(jq -r '.[] | [.name, (.ok|tostring), .detail] | @tsv' "$checks_json")

# An empty list is not a pass. A jq program that emitted nothing would
# otherwise print no lines and reach the exit as clean.
check_count="$(jq 'length' "$checks_json")"
if [[ "${check_count:-0}" -lt 1 ]]; then
  failed+=("no checks were evaluated")
fi

printf '\n  instrument wall clock: %.1fs port, %.1fs C++ baseline (mode=%s, %s shards)\n' \
  "$run_seconds" "$cpp_seconds" "$MODE" "$SHARDS"

if [[ "$MODE" == "full" ]]; then
  # By content, not by revision: `commit` can only name this run's *parent*
  # when the run produces the artifact being committed. Check one entry with
  #   tools/ci/gate-lib.sh's measured_source_digest <path>
  #
  # `$SOURCES_JSON` was taken before the build, not here -- see the snapshot
  # site for why the timing is the point. The planner `src` subtrees are in it
  # because the harnesses alone are not what produces these rates.
  jq -n --arg ts "$(date -u +%Y-%m-%dT%H:%M:%SZ)" \
        --arg stamp "$(cd "$REPO_ROOT" && git rev-parse HEAD)" \
        --arg oracle_stamp "$ORACLE_STAMP" \
        --argjson dirty "$TREE_DIRTY" \
        --argjson dirty_paths "$(printf '%s' "$DIRTY_LIST" | jq -R -s 'split("\n")|map(select(length>0))')" \
        --argjson sources "$SOURCES_JSON" \
        --argjson pins "$PINS_JSON" \
        --argjson seconds "$run_seconds" \
        --argjson cpp_seconds "$cpp_seconds" \
        --slurpfile rows "$verdict_json" \
        --slurpfile checks "$checks_json" \
     '{measured_at:$ts, commit:$stamp, oracle_stamp:$oracle_stamp, working_tree_dirty:$dirty,
       dirty_paths:$dirty_paths, measured_sources:$sources,
       mode:"full", planners:"chomp stomp", seed_base:'"$SEED_BASE"', timeout_seconds:'"$TIMEOUT_SECONDS"',
       clock_bounds:{port_seconds:'"$TIMEOUT_SECONDS"', cpp_seconds:'"$CPP_CLOCK_BOUND"',
                     note:"neither bound may be what stops a call -- both arms are meant to terminate on their iteration bound. These two need not be equal, but if one binds and the other does not, the rate difference is an instrument artifact and not a finding about the port. no-timeouts is the check for that on the port arm; the cpp arm reports a clock stop as its own error code."},
       instrument_wall_clock_seconds:$seconds,
       cpp_baseline_wall_clock_seconds:$cpp_seconds,
       regression_pins:$pins, rows:$rows,
       checks:$checks[0],
       verdict:($checks[0]|map({key:.name,
                                value:(if .ok then "PASS" else "FAIL" end)})|from_entries),
       verdict_all_pass:($checks[0]|all(.ok))}' >"$RESULTS"
  echo "  wrote $RESULTS"
else
  echo "  NOTE mode=pilot ran $PILOT_COUNT problems per config, not the $((FULL_COUNT * 4)) per planner this gate declares."
  echo "  NOTE these numbers are a harness self-test, not the Phase 8 measurement."
  echo "  NOTE run '$0 full' for the full population."
fi

echo
if [[ ${#failed[@]} -gt 0 ]]; then
  echo "FAIL ${#failed[@]} Phase 8 property check(s) failed:" >&2
  printf '  %s\n' "${failed[@]}" >&2
  exit 1
fi
echo "OK $check_count Phase 8 property checks pass (mode=$MODE)"
