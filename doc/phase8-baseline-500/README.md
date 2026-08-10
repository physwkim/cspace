# Phase 8's 500-problem baseline — both sides, committed

Every per-problem record behind this round's Phase 8 numbers: the port's CHOMP
and STOMP and upstream's own C++ CHOMP and STOMP, on the same 500 problems, at
the same per-problem seeds, with the same condition-2 grid.

## Why this directory exists

The numbers in §269.3, §269.4, §286.3 and §286.6 were computed from scratch
files that were never committed. `git ls-files` found only
`doc/phase8-condition2-stomp/`, which holds one round's *port* STOMP arms and
no C++ arm at all — so four published tables rested on files no one could
open. That is the same defect this repository has hit before in the other
direction, and the fix is the same: commit the inputs and ship a script that
regenerates every published number from them.

`rederive.py` reads **only this directory**. No oracle, no docker, no cargo, no
upstream checkout, no path outside it. A number it cannot produce is not
evidence.

    doc/phase8-baseline-500/rederive.py                     # print the report
    doc/phase8-baseline-500/rederive.py --check doc/phase8-baseline-500/rederive.txt

`rederive.txt` is that report, committed. `--check` is what
`tools/ci/check-phase8-baseline-500.sh` runs, so editing a number in the plan
that these bytes do not support fails a gate instead of sitting in prose.

## Population

`tools/ci/verify-phase8-benchmark.sh`'s own triple: panda_arm, `floor_wall`
250 problems at set seed 900001, `cage` 250 at 900002. Per-problem planner RNG
seed base 700001 on both sides (`PORT_SEED_BASE` / `PLANNER_SEED_BASE`).

The two `.set.json` files hash to §296.6's record —
`dfa126bc1b32c991ebd31f72a99c2996` and `d727fe2d7da4fade979bd2c69414d752` —
and are byte-identical to the copies in `doc/phase8-condition2-stomp/`.
`rederive.py` re-checks both hashes before it prints anything: a run on a
different population would otherwise produce a full report about different
problems. The C++ side was handed the *same file*, not a second generator run,
via the harness's `SET_FILE=`.

## Regenerating

Problem sets (byte-reproducible from the seed alone):

    cargo build --release -p cspace-planners --example plan_benchmark_problem_set
    ./target/release/examples/plan_benchmark_problem_set floor_wall 250 900001 > floor_wall.250.900001.set.json
    ./target/release/examples/plan_benchmark_problem_set cage      250 900002 > cage.250.900002.set.json

Seed-validity strata:

    GRID=0.2,0.1,0.05,0.02,0.01,0.005,0.002,0.001
    jq -c ".condition2_resolutions=[$GRID]" <set.json> \
      | ./target/release/examples/seed_validity_problem_set > seed.<config>.ndjson

The eight arms. The port side and the C++ side must be given DIFFERENT output
directories: the C++ harness copies its `SET_FILE` to
`<out_dir>/<config>.<count>.<seed>.set.json`, so a `SET_FILE` that already *is*
that path makes `cp` refuse and the arm exits 1 having planned nothing — in
seconds, which reads like a fast success in a status line.

    export CONDITION2_RESOLUTIONS=0.2,0.1,0.05,0.02,0.01,0.005,0.002,0.001
    PORT_SEED_BASE=700001 tools/ci/measure-phase8-condition2-grid.sh <planner> <config> 250 <set_seed> <port_dir>
    sg docker -c "SET_FILE=<port_dir>/<config>.250.<set_seed>.set.json \
      CONDITION2_RESOLUTIONS=$CONDITION2_RESOLUTIONS PLANNER_SEED_BASE=700001 \
      $PWD/tools/ci/measure-phase8-cpp-baseline.sh <planner> <config> 250 <set_seed> <cpp_dir>"

`sg docker`, not `docker`: an unwrapped call in this repository reports failure
as success.

## Cost

Measured on this machine while another panel was running its own STOMP arms, so
these are loaded-machine wall clocks and are a property of the machine, not of
the planners. Port CHOMP 336 s and 332 s (25 shards each); the four C++ arms 89 s
and up through the oracle container at 12 jobs; port STOMP is the long pole at
one process per problem. The C++ harness caches per-problem results under a
content-addressed key, so a re-run of an unchanged arm is near-free.

## Files

`<side>.<planner>.<config>.ndjson` — one JSON object per problem, keyed by `id`
within its config. `side` is `port` or `cpp`. `<config>.stats` is the
generator's own summary.

`seed.<config>.ndjson` is the seed-validity stratum — byte copies of the two
files already committed under `doc/phase8-condition2-stomp/`, so this directory
answers the stratum question without reaching outside itself. They were not
re-measured: a re-run of `seed_validity_problem_set` over §286's eight-point
grid agreed with the committed records on `seed_valid`, `seed_invalid_count`,
`seed_length`, `densified_waypoint_count` and the shared 0.05 grid point on
every one of the 120 problems it reached before it was stopped, and the extra
grid points it would have added are used by no published claim. Their md5s are
`33bad700bcb05af4f80961be875c9ef0` and `88800c09bcb3604d9b99f261ef6f18a3`.

`repeat.cpp.<planner>.floor_wall.ndjson` is **not an arm**. It holds two runs
of each of 21 (STOMP) / 22 (CHOMP) stratified `floor_wall` ids — `0, 13, 25,
… 249` — and nothing from `cage`. Each row is the oracle's own record
unprojected and unrenamed, plus the `planner_rng_seed` the answering process
was given and a `rep` of 1 or 2. Read it as a same-machine, same-image repeat
probe, not as a second baseline.

## How many digits are real

`§269.3` prints its medians to sixteen and seventeen significant figures. The
C++ STOMP arm does not carry that many.

The instrument for that statement is **two whole arms produced by two panels**,
not the repeat probe: `cpp.stomp.*.ndjson` here against
`../phase8-seedbase-stomp/cpp700001.*.ndjson`, an independent 500-problem cpp
STOMP run at this same seed base, over sets that are byte-identical to these.
Across all 500 ids the two agree on every solved flag, on `planner_rng_seed`
for all 500, and on the entire condition-2 grid for all 446 solved. What moves
is `length`, on 300 of the 446, worst relative `1.577e-15`; the `floor_wall`
medians are bit-identical and the `cage` medians differ by `2.056e-16`. So
about fifteen significant digits of a length survive an independent re-run.
`rederive.py` recomputes all of that and fails if the sibling directory is
absent.

The repeat probe is the corroborating measurement, at a smaller scope: two runs
of one process against one image move the length on 15 of 21 STOMP problems
(worst relative `4.890e-16`) and on 0 of 22 CHOMP problems, with no field other
than `length` differing in any pair — the whole eight-point condition-2 grid
included.

So `rederive.py` compares counts, id sets and condition-2 verdicts exactly, and
compares a median at `1e-12` relative — three orders above the measured floor,
unreachable by summation order, and far below any difference that would mean
the two runs planned differently. A cell differing above that bar, or any
difference at all in a discrete field, is a real divergence.

## Reading the condition-2 fields

Three independent verdicts per record, answering different questions:

- `condition2_valid` — at the *set's own* `motion_resolution`, 0.01 here.
- `condition2_by_resolution` — one entry per grid resolution.
- `condition2_valid_at_returned_waypoints` — no densification at all, the
  waypoints the planner actually returned.

The 0.01 grid entry and `condition2_valid` are the same bar and `rederive.py`
prints both id sets so they can be seen to agree rather than assumed to.
