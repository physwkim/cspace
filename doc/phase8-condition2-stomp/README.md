# Phase 8 condition-2 measurement for the STOMP arm

Raw measurement inputs and outputs behind PORTING-PLAN.md's STOMP-side
condition-2 section. Committed because the sweep costs 45,292 process-seconds
per arm and is not cheap to reproduce on demand.

## Population

The triple is `tools/ci/verify-phase8-benchmark.sh`'s own `CONFIGS` /
`COUNTS` / `SEEDS` arrays, read from that file rather than transcribed:
panda_arm, seed base `PORT_SEED_BASE=700001`, `floor_wall` 250 problems at
set seed 900001, `cage` 250 problems at set seed 900002. The clock argument
is the harness's non-binding `NO_CLOCK_BOUND=1e9`, so nothing here depends on
how loaded the machine was.

## Regenerating

Problem sets (byte-reproducible from the seed alone):

    cargo build --release -p moveit-planners-sbp --example plan_benchmark_problem_set
    ./target/release/examples/plan_benchmark_problem_set floor_wall 250 900001 > floor_wall.250.900001.set.json
    ./target/release/examples/plan_benchmark_problem_set cage      250 900002 > cage.250.900002.set.json

Seed-validity strata (`seed.<config>.ndjson`), which split the population into
the problems whose straight joint-space seed is already collision-free and the
discriminating remainder:

    jq -c '.condition2_resolutions=[0.05]' <set.json> \
      | ./target/release/examples/seed_validity_problem_set > seed.<config>.ndjson

The sweep itself, one invocation per config:

    CONDITION2_RESOLUTIONS=0.05 PORT_SEED_BASE=700001 \
      tools/ci/measure-phase8-condition2-grid.sh stomp floor_wall 250 900001 <out_dir>
    CONDITION2_RESOLUTIONS=0.05 PORT_SEED_BASE=700001 \
      tools/ci/measure-phase8-condition2-grid.sh stomp cage      250 900002 <out_dir>

## Files

`base.*` are the unmutated arm. `<config>.<count>.<seed>.set.json` is the
problem set the sweep consumed. `base.<config>.stats` is the generator's own
summary of that set. `base.<config>.cost.txt` is the per-problem wall-clock
distribution, derived from the shard files' mtimes.

## Reading the condition-2 fields

A record carries three independent condition-2 verdicts, and they answer
different questions. `condition2_valid` is checked at the *set's own*
`motion_resolution`, which is 0.01 for both configs here.
`condition2_by_resolution` carries one entry per requested grid resolution --
0.05 in this sweep, STOMP's upstream `COL_CHECK_DISTANCE`.
`condition2_valid_at_returned_waypoints` densifies nothing and checks the
waypoints the planner actually returned.

Because 0.05 is coarser than the set's 0.01, the grid entry can report `valid`
on a path that `condition2_valid` rejects; the two are not redundant and
neither subsumes the other. The plan section names the case where they part.
