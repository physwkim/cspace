#!/usr/bin/env python3
"""Regenerate every published number of this round's Phase 8 baseline from the
NDJSONs committed beside this file.

The round exists because §269.3, §269.4, §286.3 and §286.6 were all computed
from scratch files that were never committed: the numbers were in the plan and
nothing in the tree could check them.  So the rule here is that every number
comes from committed bytes.  This script takes no upstream checkout, no
oracle, no docker and no cargo; it reads its own directory, plus exactly one
sibling directory of committed data named by `CROSSCHECK_DIR`.  If a number
below cannot be produced from those bytes, it is not evidence and does not get
printed -- and the sibling is required rather than optional, because a
cross-check that skips itself when its input is missing prints the same "OK"
as one that ran.

    doc/phase8-baseline-500/rederive.py [--dir DIR] [--check FILE]

`--check` compares the whole report against a committed transcript
(`rederive.txt`) and exits nonzero on any difference, so a later edit to a
number in the plan that this directory does not support fails a gate rather
than sitting in prose.

The two bars are Phase 7's, quoted from §269.3 rather than recomputed here:
condition 1 is `0.9 x 498/500` and condition 3 is `1.3 x 2.6597767032746464`,
both from the C++ OMPL RRTConnect baseline §263 drew three times.  The
same-planner bars are computed from the cpp arms in this directory.
"""

import argparse
import hashlib
import json
import pathlib
import statistics
import sys

CONFIGS = (("floor_wall", 250, 900001), ("cage", 250, 900002))
PLANNERS = ("chomp", "stomp")
SIDES = ("port", "cpp")

# The per-problem planner RNG seed base both sides were run at, `PORT_SEED_BASE`
# on one and `PLANNER_SEED_BASE` on the other.  A record's own `planner_rng_seed`
# is checked against `PLANNER_SEED_BASE + id` rather than trusted.
PLANNER_SEED_BASE = 700001

# The one path outside this directory: a sibling doc directory in this same
# repository holding an independently produced cpp STOMP arm at the same seed
# base.  It is the instrument behind the precision statement below -- two whole
# arms from two panels, rather than one panel's subset run twice.
CROSSCHECK_DIR = "phase8-seedbase-stomp"

# §269.3's bars, from the C++ OMPL RRTConnect baseline §263 re-drew three
# times.  Quoted, not recomputed: no file in this directory holds an OMPL run,
# and a bar this script derived from its own inputs would move with them.
CONDITION1_RATE = 0.9 * 498 / 500
CONDITION1_COUNT = 449
CONDITION3_LIMIT = 1.3 * 2.6597767032746464

# The set's own `motion_resolution`, which is what the `condition2_valid` field
# is evaluated at.  Named here so the 0.01 row of the grid and this field can
# be shown to agree rather than assumed to.
SET_MOTION_RESOLUTION = 0.01

# What §269.3, §269.4 and §286.3 published, so this script can say whether the
# plan's cells reproduce instead of only printing numbers beside them.
PUBLISHED_269_3 = {
    ("port", "chomp"): (380, 1, 2.163978163668814),
    ("cpp", "chomp"): (370, 1, 2.1469494181135858),
    ("port", "stomp"): (441, 3, 2.210362452483207),
    ("cpp", "stomp"): (446, 2, 2.2141803455060045),
}

PUBLISHED_269_4 = {
    "chomp": {
        "counts": (363, 17, 7, 113),
        "port_only": tuple(f"floor_wall/{i}" for i in (1, 8, 136, 176, 187, 190, 197, 231))
        + tuple(f"cage/{i}" for i in (16, 30, 47, 103, 133, 141, 161, 178, 191)),
        "cpp_only": tuple(f"floor_wall/{i}" for i in (130, 203, 222))
        + tuple(f"cage/{i}" for i in (6, 33, 83, 143)),
        "reasons": ("INVALID_MOTION_PLAN", 120, 130),
    },
    "stomp": {
        "counts": (437, 4, 9, 50),
        "port_only": tuple(f"cage/{i}" for i in (133, 161, 222, 231)),
        "cpp_only": tuple(f"floor_wall/{i}" for i in (120, 221))
        + tuple(f"cage/{i}" for i in (1, 48, 91, 95, 150, 170, 200)),
        "reasons": ("PLANNING_FAILED", 59, 54),
    },
}

# §286.3's rows, in this script's column order: port chomp, cpp chomp,
# port stomp, cpp stomp.
PUBLISHED_286_3 = {
    "returned": (0, 0, 0, 0),
    0.2: (0, 0, 0, 0),
    0.1: (0, 0, 0, 0),
    0.05: (0, 0, 0, 0),
    0.02: (1, 0, 3, 2),
    0.01: (1, 1, 3, 2),
    0.005: (1, 1, 4, 3),
    0.002: (1, 1, 4, 3),
    0.001: (1, 1, 4, 3),
}

# A median is compared at a relative tolerance; a count, an id set and a
# condition-2 verdict are compared exactly.
#
# The tolerance is measured, not chosen for comfort.  `repeat.cpp.*.ndjson`
# holds one pair of runs per probe problem -- same oracle image, same
# `--planner-rng-seed`, same request bytes -- and the worst relative movement
# in a length over those pairs is 4.890e-16, with every non-length field
# identical.  A second, independent run of the whole cpp STOMP arm (p10-phase7's
# `fd42dfd3`) moved 300 of 446 lengths, worst relative 1.577e-15.  So about
# fifteen significant digits of a length survive a re-run and §269.3's
# sixteen- and seventeen-digit medians do not.  1e-12 sits three orders above
# that measured floor: it cannot be reached by summation order, and it is far
# below any difference that would mean the two runs planned differently.
MEDIAN_TOLERANCE = 1e-12

# §296.6's record of the two problem sets.  A run whose sets hash differently
# is not measuring this population, and every number below would be about a
# different 500 problems.
SET_MD5 = {
    "floor_wall.250.900001.set.json": "dfa126bc1b32c991ebd31f72a99c2996",
    "cage.250.900002.set.json": "d727fe2d7da4fade979bd2c69414d752",
}


def load(path):
    """NDJSON -> {id: record}, rejecting a duplicate id rather than merging.

    `compare-phase8-port-vs-cpp.py`'s rule and for its reason: a duplicate id
    means two shards covered one problem, which makes the population smaller
    than its line count with nothing saying so.
    """
    out = {}
    with open(path) as handle:
        for line_number, line in enumerate(handle, 1):
            line = line.strip()
            if not line:
                continue
            record = json.loads(line)
            if record["id"] in out:
                sys.exit(f"{path}:{line_number}: duplicate id {record['id']}")
            out[record["id"]] = record
    return out


def median(values):
    return statistics.median(values) if values else None


def failure_reason(record):
    """The record's failure name, with the port harness's prefix removed.

    The two sides spell the same MoveIt error differently -- the port reports
    `operation failed: INVALID_MOTION_PLAN` where the oracle reports
    `INVALID_MOTION_PLAN` -- and §269.4's histogram is over the bare names.
    Comparing the raw strings would report a difference in wording as a
    difference in behaviour.
    """
    name = record.get("failure", "?")
    prefix = "operation failed: "
    return name[len(prefix):] if name.startswith(prefix) else name


def grid_verdict(record, resolution):
    """The record's condition-2 verdict at one grid resolution, or None.

    None is returned only when the record carries no entry for that resolution
    at all; a record that carries one and is invalid returns False.  Collapsing
    the two would let a missing grid entry read as a pass.
    """
    for entry in record.get("condition2_by_resolution") or ():
        if entry["resolution"] == resolution:
            return bool(entry["valid"])
    return None


def resolutions(arms):
    """Every grid resolution present, coarse to fine, from the records."""
    seen = set()
    for records in arms.values():
        for record in records.values():
            for entry in record.get("condition2_by_resolution") or ():
                seen.add(entry["resolution"])
    return sorted(seen, reverse=True)


def report(directory, out):
    def say(line=""):
        print(line, file=out)

    # --- population -------------------------------------------------------
    say("=== population ===")
    for name, want in sorted(SET_MD5.items()):
        path = directory / name
        got = hashlib.md5(path.read_bytes()).hexdigest()
        say(f"{name}: md5 {got} {'== §296.6' if got == want else f'!= §296.6 {want}'}")
        if got != want:
            sys.exit(f"FAIL {name} is not this population's set file")
    for config, _, _ in CONFIGS:
        seed = load(directory / f"seed.{config}.ndjson")
        valid = sum(1 for r in seed.values() if r["seed_valid"])
        say(f"seed stratum {config}: seed-valid {valid}, seed-invalid {len(seed) - valid}")
    say()

    arms = {}
    for side in SIDES:
        for planner in PLANNERS:
            for config, _, _ in CONFIGS:
                arms[(side, planner, config)] = load(
                    directory / f"{side}.{planner}.{config}.ndjson"
                )

    # Both sides must cover the same ids before anything is compared: a paired
    # median over sets that are not the same problems is not paired.
    for planner in PLANNERS:
        for config, count, _ in CONFIGS:
            port = arms[("port", planner, config)]
            cpp = arms[("cpp", planner, config)]
            if port.keys() != cpp.keys():
                sys.exit(f"FAIL {planner}/{config}: the two sides cover different ids")
            if len(port) != count:
                sys.exit(f"FAIL {planner}/{config}: {len(port)} problems, not {count}")

    # Every solved record must carry every resolution §286.3 has a column for.
    # Without this the grid rows below count a *missing* entry as a pass and an
    # arm swept at one operating point reports zero violations at all of them --
    # which is not a smaller number, it is a table about a measurement that was
    # never made.  `doc/phase8-condition2-stomp/`'s arms are exactly that shape
    # (grid `[0.05]` only), so this is the concrete way to load the wrong file
    # here, not a hypothetical one.
    wanted_grid = [r for r in PUBLISHED_286_3 if r != "returned"]
    for key, records in sorted(arms.items(), key=lambda kv: str(kv[0])):
        for problem_id, record in sorted(records.items()):
            if not record.get("solved"):
                continue
            absent = [r for r in wanted_grid if grid_verdict(record, r) is None]
            if absent:
                sys.exit(
                    f"FAIL {'/'.join(key)} problem {problem_id} carries no condition-2 "
                    f"verdict at {absent}; this arm was not swept over §286's grid"
                )

    def pooled(side, planner):
        out_records = {}
        for config, _, _ in CONFIGS:
            for problem_id, record in arms[(side, planner, config)].items():
                out_records[f"{config}/{problem_id}"] = record
        return out_records

    # --- §269.3 -----------------------------------------------------------
    say("=== §269.3 four arms ===")
    say(f"condition 1 bar: {CONDITION1_RATE:.4%} = {CONDITION1_COUNT}/500")
    say(f"condition 3 bar: {CONDITION3_LIMIT!r}")
    say()
    solved_counts, length_medians = {}, {}
    for planner in PLANNERS:
        for side in SIDES:
            records = pooled(side, planner)
            solved = [r for r in records.values() if r.get("solved")]
            lengths = [r["length"] for r in solved]
            c2_fail = [
                tag
                for tag, r in sorted(records.items())
                if r.get("solved") and r.get("condition2_valid") is False
            ]
            solved_counts[(side, planner)] = len(solved)
            length_medians[(side, planner)] = median(lengths)
            say(
                f"{side} {planner}: solved {len(solved)}/500 = {len(solved) / 5:.1f}%"
                f"  condition1 {'MET' if len(solved) >= CONDITION1_COUNT else 'UNMET'}"
                f"  condition2 failures {len(c2_fail)}"
                f"  median {median(lengths)!r}"
                f"  condition3 {'MET' if median(lengths) <= CONDITION3_LIMIT else 'UNMET'}"
            )
    say()
    say(f"--- does §269.3 reproduce?  counts exact, medians within {MEDIAN_TOLERANCE:g} relative ---")
    for planner in PLANNERS:
        for side in SIDES:
            want_solved, want_c2, want_median = PUBLISHED_269_3[(side, planner)]
            records = pooled(side, planner)
            got_solved = solved_counts[(side, planner)]
            got_c2 = sum(
                1 for r in records.values()
                if r.get("solved") and r.get("condition2_valid") is False
            )
            got_median = length_medians[(side, planner)]
            drift = abs(got_median - want_median) / want_median
            ok = got_solved == want_solved and got_c2 == want_c2 and drift <= MEDIAN_TOLERANCE
            say(
                f"{side} {planner}: solved {got_solved} vs {want_solved}"
                f"  condition2 {got_c2} vs {want_c2}"
                f"  median {got_median!r} vs {want_median!r} (relative {drift:.3e})"
                f"  -> {'REPRODUCES' if ok else 'DOES NOT REPRODUCE'}"
            )
    say()
    say("--- same-planner bars (§269.3's second table) ---")
    for planner in PLANNERS:
        cpp_solved = solved_counts[("cpp", planner)]
        port_solved = solved_counts[("port", planner)]
        bar1 = 0.9 * cpp_solved / 500
        bar3 = 1.3 * length_medians[("cpp", planner)]
        say(
            f"port {planner} condition1: {port_solved / 5:.4f}% vs bar "
            f"0.9 x {cpp_solved}/500 = {bar1:.4%} -> "
            f"{'MET' if port_solved / 500 >= bar1 else 'UNMET'}"
        )
        say(
            f"port {planner} condition3: {length_medians[('port', planner)]!r} vs bar "
            f"1.3 x {length_medians[('cpp', planner)]!r} = {bar3!r} -> "
            f"{'MET' if length_medians[('port', planner)] <= bar3 else 'UNMET'}"
        )
    say()

    # --- §269.4 -----------------------------------------------------------
    say("=== §269.4 four-way split ===")
    for planner in PLANNERS:
        both, port_only, cpp_only, neither = [], [], [], []
        port_reasons, cpp_reasons = {}, {}
        for config, _, _ in CONFIGS:
            port = arms[("port", planner, config)]
            cpp = arms[("cpp", planner, config)]
            for problem_id in sorted(port):
                tag = f"{config}/{problem_id}"
                p, c = port[problem_id], cpp[problem_id]
                if p.get("solved") and c.get("solved"):
                    both.append(tag)
                elif p.get("solved"):
                    port_only.append(tag)
                elif c.get("solved"):
                    cpp_only.append(tag)
                else:
                    neither.append(tag)
                if not p.get("solved"):
                    key = failure_reason(p)
                    port_reasons[key] = port_reasons.get(key, 0) + 1
                if not c.get("solved"):
                    key = failure_reason(c)
                    cpp_reasons[key] = cpp_reasons.get(key, 0) + 1
        say(
            f"{planner}: both {len(both)}  port-only {len(port_only)}  "
            f"cpp-only {len(cpp_only)}  neither {len(neither)}  "
            f"disagreements {len(port_only) + len(cpp_only)}"
        )
        say(f"{planner} port-only ids: {' '.join(port_only) if port_only else '(none)'}")
        say(f"{planner} cpp-only ids : {' '.join(cpp_only) if cpp_only else '(none)'}")
        say(f"{planner} port failure reasons: {dict(sorted(port_reasons.items()))}")
        say(f"{planner} cpp  failure reasons: {dict(sorted(cpp_reasons.items()))}")
        # Every quantity here is discrete, so every comparison is exact.
        want = PUBLISHED_269_4[planner]
        got_counts = (len(both), len(port_only), len(cpp_only), len(neither))
        checks = [
            ("counts", got_counts == want["counts"], f"{got_counts} vs {want['counts']}"),
            ("port-only ids", set(port_only) == set(want["port_only"]),
             f"{len(port_only)} vs {len(want['port_only'])}"),
            ("cpp-only ids", set(cpp_only) == set(want["cpp_only"]),
             f"{len(cpp_only)} vs {len(want['cpp_only'])}"),
        ]
        reason, want_port_n, want_cpp_n = want["reasons"]
        checks.append(
            ("failure reasons",
             port_reasons == {reason: want_port_n} and cpp_reasons == {reason: want_cpp_n},
             f"port {port_reasons} vs {{{reason!r}: {want_port_n}}}, "
             f"cpp {cpp_reasons} vs {{{reason!r}: {want_cpp_n}}}")
        )
        for name, ok, detail in checks:
            say(f"{planner} {name}: {detail} -> {'REPRODUCES' if ok else 'DOES NOT REPRODUCE'}")
    say()

    # --- §286.3 -----------------------------------------------------------
    say("=== §286.3 condition-2 failures per resolution ===")
    grid = resolutions(arms)
    header = "resolution      " + "".join(
        f"{side} {planner:<6}".ljust(18) for planner in PLANNERS for side in SIDES
    )
    say(header)
    returned_row = "returned        "
    for planner in PLANNERS:
        for side in SIDES:
            records = pooled(side, planner)
            fails = sum(
                1
                for r in records.values()
                if r.get("solved") and r.get("condition2_valid_at_returned_waypoints") is False
            )
            returned_row += str(fails).ljust(18)
    say(returned_row)
    failing_ids = {}
    for resolution in grid:
        row = f"{resolution:<16}"
        for planner in PLANNERS:
            for side in SIDES:
                records = pooled(side, planner)
                bad = [
                    tag
                    for tag, r in sorted(records.items())
                    if r.get("solved") and grid_verdict(r, resolution) is False
                ]
                failing_ids[(side, planner, resolution)] = bad
                row += str(len(bad)).ljust(18)
        say(row)
    say()
    say("--- does §286.3 reproduce?  every cell is a count, so every check is exact ---")
    for label, want in PUBLISHED_286_3.items():
        got = []
        for planner in PLANNERS:
            for side in SIDES:
                records = pooled(side, planner)
                if label == "returned":
                    got.append(sum(
                        1 for r in records.values()
                        if r.get("solved")
                        and r.get("condition2_valid_at_returned_waypoints") is False
                    ))
                else:
                    got.append(len(failing_ids[(side, planner, label)]))
        # PUBLISHED_286_3's column order is port chomp, cpp chomp, port stomp,
        # cpp stomp -- the same order the loops above walk.
        got = tuple(got)
        say(
            f"{label}: {got} vs {want} -> "
            f"{'REPRODUCES' if got == want else 'DOES NOT REPRODUCE'}"
        )
    say()
    say("--- failing ids, every arm, every resolution where the set is nonempty ---")
    for (side, planner, resolution), bad in sorted(
        failing_ids.items(), key=lambda kv: (kv[0][1], kv[0][0], -kv[0][2])
    ):
        if bad:
            say(f"{side} {planner} @ {resolution}: {' '.join(bad)}")
    say()
    say("--- the set's own motion_resolution field vs the grid's matching row ---")
    for planner in PLANNERS:
        for side in SIDES:
            records = pooled(side, planner)
            field = {
                tag
                for tag, r in sorted(records.items())
                if r.get("solved") and r.get("condition2_valid") is False
            }
            grid_set = set(failing_ids[(side, planner, SET_MOTION_RESOLUTION)])
            say(
                f"{side} {planner}: condition2_valid fails {sorted(field)} ; "
                f"grid @ {SET_MOTION_RESOLUTION} fails {sorted(grid_set)} ; "
                f"{'agree' if field == grid_set else 'DISAGREE'}"
            )
    say()

    # --- condition3-paired ------------------------------------------------
    # The check §264.12's bullet names and no section has ever published on
    # this population.  §269.3's condition 3 is each side's median over ITS OWN
    # solved set, which lets a port that fails the hard problems drop their long
    # paths out of its own median -- passing more easily the worse it gets.
    # This median is over the problems BOTH sides solved, so neither side can
    # choose its population.
    say("=== condition3-paired (median over the problems both sides solved) ===")
    for planner in PLANNERS:
        rows = [(config, arms[("port", planner, config)], arms[("cpp", planner, config)])
                for config, _, _ in CONFIGS]
        rows.append(("pooled", pooled("port", planner), pooled("cpp", planner)))
        for label, port, cpp in rows:
            paired = [k for k in sorted(port) if port[k].get("solved") and cpp[k].get("solved")]
            port_median = median([port[k]["length"] for k in paired])
            cpp_median = median([cpp[k]["length"] for k in paired])
            if not paired or cpp_median is None:
                say(f"{planner}/{label}: no paired problems -- not measurable")
                continue
            limit = 1.3 * cpp_median
            say(
                f"{planner}/{label}: n={len(paired)}  port {port_median!r}  "
                f"cpp {cpp_median!r}  limit 1.3x = {limit!r}  "
                f"ratio {port_median / cpp_median:.6f}x  "
                f"{'MET' if port_median <= limit else 'UNMET'}"
            )
    say()

    # --- condition3-paired, powered subset --------------------------------
    # A paired median over all both-solved problems is dominated by problems
    # neither optimizer moved: where the straight-line seed trajectory is
    # already collision-free, both sides return it and the comparison carries
    # no information about either optimizer.  §286.1's seed stratum names those
    # problems, and on the port CHOMP arm the stratum coincides exactly with
    # the port's own account of having accepted no update, which is printed
    # below as a check on the criterion rather than an assumption about it.
    say("=== condition3-paired, restricted to the seed-invalid stratum ===")
    seed_valid_ids = set()
    for config, _, _ in CONFIGS:
        for problem_id, record in load(directory / f"seed.{config}.ndjson").items():
            if record["seed_valid"]:
                seed_valid_ids.add(f"{config}/{problem_id}")
    for planner in PLANNERS:
        port, cpp = pooled("port", planner), pooled("cpp", planner)
        paired = [k for k in sorted(port) if port[k].get("solved") and cpp[k].get("solved")]
        powered = [k for k in paired if k not in seed_valid_ids]
        port_median = median([port[k]["length"] for k in powered])
        cpp_median = median([cpp[k]["length"] for k in powered])
        if cpp_median is None:
            say(f"{planner}/pooled: no seed-invalid paired problems -- not measurable")
            continue
        longer = sum(1 for k in powered if port[k]["length"] > cpp[k]["length"])
        say(
            f"{planner}/pooled: n={len(powered)} of {len(paired)} paired  "
            f"port {port_median!r}  cpp {cpp_median!r}  "
            f"limit 1.3x = {1.3 * cpp_median!r}  ratio {port_median / cpp_median:.6f}x  "
            f"{'MET' if port_median <= 1.3 * cpp_median else 'UNMET'}"
        )
        say(f"{planner}/pooled: port longer on {longer}, cpp longer on {len(powered) - longer}")
    say()
    say("--- the criterion, checked against the port CHOMP arm's own loop counter ---")
    agree, disagree = 0, 0
    for config, _, _ in CONFIGS:
        for problem_id, record in arms[("port", "chomp", config)].items():
            if not record.get("solved"):
                continue
            idle = record["loop"]["accepted"] == 0
            if idle == (f"{config}/{problem_id}" in seed_valid_ids):
                agree += 1
            else:
                disagree += 1
    say(
        f"seed_valid <-> port CHOMP accepted no update: agree {agree}, disagree {disagree}"
        f" ({'exact' if not disagree else 'NOT exact'})"
    )
    say()

    # --- no-regression-cpp-solved -----------------------------------------
    say("=== no-regression-cpp-solved (the C++ solved counts a floor would pin) ===")
    for planner in PLANNERS:
        for config, count, _ in CONFIGS:
            cpp = arms[("cpp", planner, config)]
            n = sum(1 for r in cpp.values() if r.get("solved"))
            say(f"cpp {planner}/{config}: {n}/{count}")
        say(f"cpp {planner}/pooled: {solved_counts[('cpp', planner)]}/500")
    say()

    # --- §300.9's named claim ---------------------------------------------
    # main's 48ab9f8b closes §300.9's three-violation bullet on this pair of
    # id sets.  The port side was re-derivable from doc/phase8-condition2-stomp;
    # the cpp side rested on a transcribed table with no committed file behind
    # it, which is the defect this directory exists to remove.
    say("=== §300.9's named claim, at the 0.01 bar ===")
    cpp_stomp = failing_ids[("cpp", "stomp", SET_MOTION_RESOLUTION)]
    port_stomp = failing_ids[("port", "stomp", SET_MOTION_RESOLUTION)]
    say(f"cpp STOMP  @ 0.01: {' '.join(cpp_stomp) if cpp_stomp else '(none)'}")
    say(f"port STOMP @ 0.01: {' '.join(port_stomp) if port_stomp else '(none)'}")
    overlap = sorted(set(cpp_stomp) & set(port_stomp))
    say(f"disjoint @ 0.01: {'yes' if not overlap else f'no, shared {overlap}'}")
    # §300.9 also claims disjointness holds when the grid is widened all the
    # way down, not only at 0.01, so the union over every resolution is the
    # claim's actual scope and is checked as such.
    cpp_any, port_any = set(), set()
    for resolution in grid:
        cpp_any |= set(failing_ids[("cpp", "stomp", resolution)])
        port_any |= set(failing_ids[("port", "stomp", resolution)])
    say(f"cpp STOMP  over the whole grid: {' '.join(sorted(cpp_any)) or '(none)'}")
    say(f"port STOMP over the whole grid: {' '.join(sorted(port_any)) or '(none)'}")
    both_any = sorted(cpp_any & port_any)
    say(f"disjoint over the whole grid: {'yes' if not both_any else f'no, shared {both_any}'}")
    # And the stratum claim the same bullet makes: every condition-2 failure of
    # all four arms comes from the seed-invalid stratum.
    leaked = set()
    for (side, planner, _resolution), bad in failing_ids.items():
        leaked |= set(bad) & seed_valid_ids
    say(
        "every condition-2 failure is in the seed-invalid stratum: "
        f"{'yes' if not leaked else f'no, seed-valid failures {sorted(leaked)}'}"
    )
    say()

    # --- how far these numbers are reproducible ---------------------------
    # §269.3 prints its medians to 17 significant figures.  The C++ STOMP arm
    # does not carry that many: re-running one problem through the same oracle
    # image, at the same `--planner-rng-seed`, on the same request bytes,
    # returns a trajectory whose length moves in the last ulp.  The two repeat
    # files hold one such pair per problem, so the claim is a count over
    # committed bytes rather than an assertion about a run nobody kept.  Every
    # other field is compared too: what moves is the length alone, which is why
    # the solved sets, the split and the condition-2 verdicts all reproduce
    # exactly while the 17th digit of a median does not.
    say("=== repeat runs: how many digits of a length are reproducible ===")
    for planner in PLANNERS:
        path = directory / f"repeat.cpp.{planner}.floor_wall.ndjson"
        if not path.exists():
            say(f"cpp {planner}: no repeat file")
            continue
        reps = {}
        with open(path) as handle:
            for line in handle:
                if line.strip():
                    record = json.loads(line)
                    reps.setdefault(record["id"], {})[record["rep"]] = record
        same = differ = other = unpinned = 0
        worst = 0.0
        for problem_id, pair in sorted(reps.items()):
            first, second = pair[1], pair[2]
            # The seed each process was given, as the oracle recorded it.
            # Without this the pair shows two runs agreeing and says nothing
            # about whether they were even asked the same question.
            if not (
                first["planner_rng_seed"] == second["planner_rng_seed"]
                == PLANNER_SEED_BASE + problem_id
            ):
                unpinned += 1
            if first["length"] == second["length"]:
                same += 1
            else:
                differ += 1
                worst = max(worst, abs(first["length"] - second["length"]) / second["length"])
            rest = {k: v for k, v in first.items() if k not in ("rep", "length")}
            if rest != {k: v for k, v in second.items() if k not in ("rep", "length")}:
                other += 1
        say(
            f"cpp {planner}: {len(reps)} problems run twice -- "
            f"length identical {same}, length differs {differ} "
            f"(worst relative {worst:.3e}); records differing in any other field "
            f"(including the whole condition-2 grid): {other}; "
            f"rows whose recorded seed is not {PLANNER_SEED_BASE}+id: {unpinned}"
        )
        say(
            f"cpp {planner}: ids {sorted(reps)} -- a stratified subset of the "
            f"{CONFIGS[0][0]} arm, not the arm"
        )
    say()

    # The digit count above rests on a 21-problem subset of one config, run
    # twice on one machine against one image.  The stronger instrument is two
    # *whole* arms produced by two panels: `doc/phase8-seedbase-stomp/`'s
    # `cpp700001.*` is an independent 500-problem cpp STOMP run at this same
    # seed base.  Required, not optional -- a cross-check that quietly skips
    # itself when its input is absent reports the same "OK" as one that ran.
    say("=== cross-check: this cpp STOMP arm against an independently produced one ===")
    sibling = directory.parent / CROSSCHECK_DIR
    if not sibling.is_dir():
        sys.exit(f"FAIL {sibling} is absent; the cross-check cannot be skipped")
    for config, _, _ in CONFIGS:
        mine = arms[("cpp", "stomp", config)]
        theirs = load(sibling / f"cpp700001.{config}.ndjson")
        if mine.keys() != theirs.keys():
            sys.exit(f"FAIL {config}: the two arms cover different ids")
        seeds = sum(
            1 for i in mine
            if mine[i].get("planner_rng_seed")
            == theirs[i].get("planner_rng_seed")
            == PLANNER_SEED_BASE + i
        )
        disagree = [i for i in sorted(mine) if mine[i]["solved"] != theirs[i]["solved"]]
        solved = [i for i in sorted(mine) if mine[i]["solved"]]
        grids = sum(
            1 for i in solved
            if mine[i].get("condition2_by_resolution") == theirs[i].get("condition2_by_resolution")
        )
        identical = [i for i in solved if mine[i]["length"] == theirs[i]["length"]]
        worst = max(
            (abs(mine[i]["length"] - theirs[i]["length"]) / theirs[i]["length"] for i in solved),
            default=0.0,
        )
        mine_median = median([mine[i]["length"] for i in solved])
        their_median = median([theirs[i]["length"] for i in solved])
        drift = 0.0 if mine_median == their_median else abs(mine_median - their_median) / their_median
        say(
            f"{config}: ids {len(mine)}  solved-flag disagreements {len(disagree)}  "
            f"seeds pinned to {PLANNER_SEED_BASE}+id on both {seeds}/{len(mine)}  "
            f"condition-2 grids identical {grids}/{len(solved)}"
        )
        say(
            f"{config}: length identical {len(identical)}/{len(solved)}, "
            f"worst relative {worst:.3e}; median {mine_median!r} vs {their_median!r} "
            f"(relative {drift:.3e}, {'within' if drift <= MEDIAN_TOLERANCE else 'ABOVE'} "
            f"{MEDIAN_TOLERANCE:g})"
        )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--dir", default=str(pathlib.Path(__file__).resolve().parent))
    parser.add_argument("--check")
    args = parser.parse_args()

    import io

    buffer = io.StringIO()
    report(pathlib.Path(args.dir), buffer)
    text = buffer.getvalue()

    if args.check:
        want = pathlib.Path(args.check).read_text()
        if want != text:
            sys.stdout.write(text)
            print(f"FAIL {args.check} does not match this run", file=sys.stderr)
            import difflib

            for line in difflib.unified_diff(
                want.splitlines(), text.splitlines(), "committed", "this run", lineterm=""
            ):
                print(line, file=sys.stderr)
            return 1
        print(f"OK {args.check} reproduces from the NDJSONs beside it")
        return 0

    sys.stdout.write(text)
    return 0


if __name__ == "__main__":
    sys.exit(main())
