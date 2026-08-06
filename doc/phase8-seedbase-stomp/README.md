# STOMP's second planner seed base — Phase 8 condition-2 evidence

Raw inputs and outputs behind PORTING-PLAN.md's section on whether STOMP's
condition-2 failures move when only the planner RNG seed base moves.
`doc/phase8-condition2-stomp/` holds the same measurement at one seed base;
this directory adds the second, plus an upstream column at the first that is
re-measured here rather than transcribed.

## Population

The same 500 problems, byte for byte: panda_arm, `floor_wall` 250 problems at
set seed 900001 and `cage` 250 at set seed 900002. `*.set.json` here is
`cmp`-identical to `doc/phase8-condition2-stomp/`'s (md5
`dfa126bc1b32c991ebd31f72a99c2996` and `d727fe2d7da4fade979bd2c69414d752`),
which is what makes the two seed bases comparable at all.

What differs between the arms is one number: the per-problem planner RNG seed,
`seed_base + problem.id`. `PORT_SEED_BASE` on the port side,
`PLANNER_SEED_BASE` (the oracle's `--planner-rng-seed`) on the C++ side.

The clock is non-binding on both sides — the port harness's `NO_CLOCK_BOUND`
= 1e9 and the oracle's `STOMP_CLOCK_BOUND` = 3600 with `timed_out` asserted
zero — so nothing here depends on how loaded the machine was. It was loaded:
the two port arms ran concurrently, 500 single-problem processes on 96 cores.

## Files

| file | arm |
| --- | --- |
| `port.stomp.<config>.ndjson` | this port's STOMP, seed base **424242** |
| `stomp.<config>.ndjson` | upstream C++ STOMP, seed base **424242** |
| `cpp700001.<config>.ndjson` | upstream C++ STOMP, seed base **700001** |
| `<config>.<count>.<seed>.set.json` | the problem set all six arms consumed |
| `<config>.stats` | the generator's own summary of that set |
| `rederive.txt` | `rederive.py`'s output, committed so a reader can diff |

The port arm at seed base 700001 is not duplicated here: it is
`doc/phase8-condition2-stomp/base.port.stomp.<config>.ndjson`, and its grid is
`[0.05]` rather than the eight-point grid used here. `rederive.py` therefore
takes that arm's id set from PORTING-PLAN.md §286.3 and re-measures only the
C++ side of that column — the mismatch, if the transcription were wrong, is
printed rather than assumed away.

Seed validity (`seed.<config>.ndjson`) is likewise not duplicated: it is a
property of the problem, not of a planner seed, so `rederive.py` reads
`doc/phase8-condition2-stomp/`'s copies.

## Regenerating

    G=0.2,0.1,0.05,0.02,0.01,0.005,0.002,0.001

    PORT_SEED_BASE=424242 CONDITION2_RESOLUTIONS=$G \
      tools/ci/measure-phase8-condition2-grid.sh stomp floor_wall 250 900001 <out>
    PORT_SEED_BASE=424242 CONDITION2_RESOLUTIONS=$G \
      tools/ci/measure-phase8-condition2-grid.sh stomp cage       250 900002 <out>

    PLANNER_SEED_BASE=424242 CONDITION2_RESOLUTIONS=$G \
      SET_FILE=<out>/floor_wall.250.900001.set.json \
      tools/ci/measure-phase8-cpp-baseline.sh stomp floor_wall 250 900001 <cpp_out>
    PLANNER_SEED_BASE=424242 CONDITION2_RESOLUTIONS=$G \
      SET_FILE=<out>/cage.250.900002.set.json \
      tools/ci/measure-phase8-cpp-baseline.sh stomp cage       250 900002 <cpp_out>

and the same two C++ invocations with `PLANNER_SEED_BASE=700001` for the
`cpp700001.*` files. Then:

    doc/phase8-seedbase-stomp/rederive.py

## Reading the condition-2 fields

Unchanged from `doc/phase8-condition2-stomp/README.md`: `condition2_valid` is
the verdict at the set's own `motion_resolution` (0.01 here),
`condition2_by_resolution` carries one entry per grid resolution, and
`condition2_valid_at_returned_waypoints` densifies nothing.

One consequence of `densify`'s `steps = ceil(distance / resolution)` rule is
worth stating, because this grid makes it visible: a finer resolution's sample
set does **not** contain a coarser one's. For a segment of length 0.15, 0.02
samples at `i/8` and 0.01 at `i/15`, which share only the endpoints. So a
problem can fail at 0.02 and pass at 0.01, and one in this data does. Per-level
counts are therefore not monotone in the resolution, and nothing here assumes
they are.
