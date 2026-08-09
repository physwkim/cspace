#!/usr/bin/env python3
"""Condition 2 for the CHOMP/STOMP class: port against the same planner's C++.

PORTING-PLAN.md's Phase 8 row leaves condition 2 unspecified for this planner
class because upstream's own C++ CHOMP and STOMP do not reach 100% at Phase
7's `motion_resolution` = 0.01 either, so a miss there cannot separate a port
defect from the class's own behaviour. This reads the two sides' sweeps
(`measure-phase8-condition2-grid.sh` and `measure-phase8-cpp-baseline.sh`, run
with the same `CONDITION2_RESOLUTIONS`) and reports, per planner:

  * how the problem population splits by whether the straight joint-space seed
    both planners start from was ALREADY collision-free
    (`seed_validity_problem_set.rs`), and what each side did on each stratum;
  * each side's condition-2 pass rate at every resolution in the grid, and at
    the planner's own returned waypoints, on the full population and on the
    seed-invalid stratum alone;
  * the PAIRED table -- `b` = port fails where C++ passes, `c` the reverse --
    with McNemar's `|b - c| / sqrt(b + c)` and whether `b + c` clears
    `MINIMUM_USABLE_B_PLUS_C`. A paired gate whose `b + c` is below that
    cannot resolve a divergence, and `moveit-diff` calls that `Underpowered`
    rather than a pass;
  * `r*`, the finest grid resolution at which the C++ implementation of the
    SAME planner reaches 100% over its own solved problems.

`r*` is what makes a specified condition possible. It is defined by the C++
side alone and never looks at the port's numbers, so it cannot be moved to
make the port pass: the bar sits exactly where upstream's own implementation
of the same algorithm meets it, and a port miss there is a port defect rather
than a property of trajectory optimisers.

`--seed` is REQUIRED, and that is the point. Half of this population's
problems have a collision-free straight seed, and on those CHOMP and STOMP
return that seed unchanged -- so their condition-2 verdict is a property of
the problem generator, identical on both sides by construction, and a rate
that mixes them in is not a measurement of either implementation. An
instrument that could print such a rate without naming the split would be
reporting the generator under the port's name.

Usage:
  analyse-phase8-condition2-grid.py --planner chomp \
      --config floor_wall --port a.ndjson --cpp b.ndjson --seed s1.ndjson \
      --config cage --port c.ndjson --cpp d.ndjson --seed s2.ndjson
"""

import argparse
import json
import math
import sys

# `tools/moveit-diff/src/main.rs`'s constant, not a second opinion: the paired
# gate this repo already calibrated calls `b + c` below it `Underpowered`, and
# a Phase 8 gate that used a laxer number would be claiming power the same
# statistic was refused elsewhere in the tree.
MINIMUM_USABLE_B_PLUS_C = 61
# `PAIRED_DIVERGENCE_Z_THRESHOLD`, same file, same reason.
PAIRED_DIVERGENCE_Z_THRESHOLD = 3.0

# What "no densification at all" is called in the per-problem records: the
# planner's own returned waypoints, the only resolution at which either
# implementation makes a claim (`chomp_planner.cpp:283` decides SUCCESS on
# `optimizer->isCollisionFree()` over its own 101 points).
RETURNED = "returned-waypoints"

# Relative tolerance for "the returned path is the seed": both sides' `length`
# and the seed's `seed_length` are the same `StateSpace::distance` sum over the
# same waypoints, so an untouched seed reproduces to rounding, not to a margin.
SEED_LENGTH_RELATIVE_TOLERANCE = 1e-9


def paired_divergence_z(b, c):
    """McNemar's normal approximation, `paired_divergence_z`'s exact rule."""
    if b + c == 0:
        return 0.0
    return abs(b - c) / math.sqrt(b + c)


def load(path):
    records = {}
    with open(path) as handle:
        for line_number, line in enumerate(handle, 1):
            line = line.strip()
            if not line:
                continue
            record = json.loads(line)
            key = record["id"]
            if key in records:
                sys.exit(f"{path}:{line_number}: duplicate id {key}")
            records[key] = record
    return records


def levels(record):
    """`{level: valid}` for one solved record, `None` when it did not solve.

    A solved record with no `condition2_by_resolution` is an error rather than
    an empty grid: it means the sweep ran without `CONDITION2_RESOLUTIONS` and
    every level below would silently be missing from the population.
    """
    if not record.get("solved"):
        return None
    grid = record.get("condition2_by_resolution")
    if grid is None:
        sys.exit(f"solved record id {record['id']} carries no condition2_by_resolution")
    out = {RETURNED: bool(record["condition2_valid_at_returned_waypoints"])}
    for entry in grid:
        out[float(entry["resolution"])] = bool(entry["valid"])
    return out


def level_name(level):
    return level if isinstance(level, str) else f"{level:g}"


def table(order, rows, port_denominator, cpp_denominator, title):
    print(f"--- {title} ---")
    header = ("level", "port valid", "cpp valid", "b(port-)", "c(cpp-)", "b+c", "|z|", "power")
    print("{:<18}{:>14}{:>14}{:>10}{:>9}{:>6}{:>7}  {}".format(*header))
    for level in order:
        row = rows[level]
        print("{:<18}{:>14}{:>14}{:>10}{:>9}{:>6}{:>7.2f}  {}".format(
            level_name(level),
            f"{row['port_valid']}/{port_denominator}",
            f"{row['cpp_valid']}/{cpp_denominator}",
            row["b"], row["c"], row["b"] + row["c"], row["z"],
            "POWERED" if row["powered"] else "underpowered"))


def build_rows(order, port_levels, cpp_levels, port_solved, cpp_solved, both):
    rows = {}
    for level in order:
        b = sum(1 for t in both if not port_levels[t][level] and cpp_levels[t][level])
        c = sum(1 for t in both if port_levels[t][level] and not cpp_levels[t][level])
        rows[level] = dict(
            port_valid=sum(1 for t in port_solved if port_levels[t][level]),
            cpp_valid=sum(1 for t in cpp_solved if cpp_levels[t][level]),
            b=b, c=c, z=paired_divergence_z(b, c),
            powered=(b + c) >= MINIMUM_USABLE_B_PLUS_C)
    return rows


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--planner", required=True)
    parser.add_argument("--config", action="append", required=True)
    parser.add_argument("--port", action="append", required=True)
    parser.add_argument("--cpp", action="append", required=True)
    parser.add_argument("--seed", action="append", required=True,
                        help="seed_validity_problem_set NDJSON for this config; required, "
                             "see this script's docstring")
    args = parser.parse_args()
    if not (len(args.config) == len(args.port) == len(args.cpp) == len(args.seed)):
        sys.exit("--config/--port/--cpp/--seed must be repeated the same number of times")

    port_levels, cpp_levels, seed_valid, seed_length = {}, {}, {}, {}
    total = 0
    per_config_seed = []
    for config, port_path, cpp_path, seed_path in zip(
            args.config, args.port, args.cpp, args.seed):
        port, cpp, seeds = load(port_path), load(cpp_path), load(seed_path)
        if port.keys() != cpp.keys():
            sys.exit(f"{config}: the two sides do not cover the same problems")
        if seeds.keys() != port.keys():
            sys.exit(f"{config}: the seed sweep does not cover the same problems")
        total += len(port)
        valid_here = 0
        for problem_id in sorted(port):
            tag = f"{config}/{problem_id}"
            port_levels[tag] = levels(port[problem_id])
            cpp_levels[tag] = levels(cpp[problem_id])
            seed_valid[tag] = bool(seeds[problem_id]["seed_valid"])
            seed_length[tag] = float(seeds[problem_id]["seed_length"])
            valid_here += seed_valid[tag]
        per_config_seed.append((config, valid_here, len(port)))

    port_solved = [t for t in port_levels if port_levels[t] is not None]
    cpp_solved = [t for t in cpp_levels if cpp_levels[t] is not None]
    both = [t for t in port_solved if cpp_levels[t] is not None]

    grids = {frozenset(v) for v in list(port_levels.values()) + list(cpp_levels.values()) if v}
    if len(grids) != 1:
        sys.exit(f"the two sides walked {len(grids)} different grids; they must match")
    # Coarse (least strict) first, with the returned waypoints -- no
    # densification at all -- as the coarse limit.
    order = [RETURNED] + sorted((g for g in next(iter(grids)) if g != RETURNED), reverse=True)

    print(f"=== phase 8 condition 2, {args.planner}: n={total} problems ===")
    print(f"port solved {len(port_solved)}/{total}   cpp solved {len(cpp_solved)}/{total}   "
          f"both solved {len(both)}")
    print()

    # ---- population -------------------------------------------------------
    # Every rate below is reported against this split, because on the
    # seed-valid stratum both implementations return the seed and the verdict
    # is the problem generator's, not the optimizer's.
    all_valid = sum(seed_valid.values())
    print("population: which problems already had a collision-free straight seed")
    for config, valid_here, n_here in per_config_seed:
        print(f"  {config:<12} seed-valid {valid_here}/{n_here}")
    print(f"  {'TOTAL':<12} seed-valid {all_valid}/{total} "
          f"({100.0 * all_valid / total:.1f}%)")
    for name, solved, records in (("port", port_solved, port_levels),
                                  ("cpp", cpp_solved, cpp_levels)):
        sv_solved = sum(1 for t in solved if seed_valid[t])
        si_solved = len(solved) - sv_solved
        print(f"  {name} solved: {sv_solved}/{all_valid} of the seed-valid stratum, "
              f"{si_solved}/{total - all_valid} of the seed-invalid stratum")
    print()

    # ---- is the returned path the seed? -----------------------------------
    # The claim "on the seed-valid stratum the optimizer did nothing" is
    # measured here rather than argued: an untouched seed has exactly the
    # seed's joint-space length.
    for name, solved, path_records in (("port", port_solved, args.port),
                                       ("cpp", cpp_solved, args.cpp)):
        lengths = {}
        for config, ndjson in zip(args.config, path_records):
            for problem_id, record in load(ndjson).items():
                if record.get("solved"):
                    lengths[f"{config}/{problem_id}"] = float(record["length"])
        stratum = [t for t in solved if seed_valid[t]]
        same = sum(1 for t in stratum
                   if abs(lengths[t] / seed_length[t] - 1.0) < SEED_LENGTH_RELATIVE_TOLERANCE)
        print(f"  {name}: returned path length equals its seed's to "
              f"{SEED_LENGTH_RELATIVE_TOLERANCE:g} relative on {same}/{len(stratum)} "
              f"of the seed-valid problems it solved")
    print()

    rows = build_rows(order, port_levels, cpp_levels, port_solved, cpp_solved, both)
    table(order, rows, len(port_solved), len(cpp_solved),
          f"full population, n={total}")
    print()

    # ---- the discriminating stratum ---------------------------------------
    si_port = [t for t in port_solved if not seed_valid[t]]
    si_cpp = [t for t in cpp_solved if not seed_valid[t]]
    si_both = [t for t in both if not seed_valid[t]]
    si_rows = build_rows(order, port_levels, cpp_levels, si_port, si_cpp, si_both)
    table(order, si_rows, len(si_port), len(si_cpp),
          f"seed-invalid stratum only, n={total - all_valid} "
          f"(port solved {len(si_port)}, cpp solved {len(si_cpp)}, both {len(si_both)})")
    print()

    # The seed-valid stratum is where the two sides cannot differ. Printing the
    # count rather than asserting it: a nonzero here would be a real finding
    # (two implementations disagreeing about a path they both left alone).
    sv_both = [t for t in both if seed_valid[t]]
    sv_disagree = sum(1 for t in sv_both for lv in order
                      if port_levels[t][lv] != cpp_levels[t][lv])
    print(f"seed-valid stratum: port and cpp disagree at {sv_disagree} of "
          f"{len(sv_both) * len(order)} level-verdicts over the {len(sv_both)} problems "
          f"both solved. This stratum cannot discriminate the two implementations.")
    print()

    # ---- r* ---------------------------------------------------------------
    perfect = [lv for lv in order if rows[lv]["cpp_valid"] == len(cpp_solved)]
    if not perfect:
        print("r*: NONE -- the C++ side is not 100% at any level in this grid, "
              "not even at its own returned waypoints. No specifiable bar exists here.")
        return 2
    r_star = perfect[-1]
    print(f"r* = {level_name(r_star)}: the finest level at which C++ {args.planner} is "
          f"{rows[r_star]['cpp_valid']}/{len(cpp_solved)} = 100%.")
    finer = order[order.index(r_star) + 1:]
    if finer:
        print(f"     one level finer ({level_name(finer[0])}) C++ {args.planner} is "
              f"{rows[finer[0]]['cpp_valid']}/{len(cpp_solved)}, so the bar cannot be "
              f"set there.")
    port_at = rows[r_star]["port_valid"]
    verdict = "MET" if port_at == len(port_solved) else "UNMET"
    print(f"condition 2 at r*: port {args.planner} {port_at}/{len(port_solved)} "
          f"-> {verdict}")

    powered_levels = [lv for lv in order if rows[lv]["powered"]]
    if powered_levels:
        print(f"paired gate: powered at {len(powered_levels)} level(s); "
              + ", ".join(
                  f"{level_name(lv)} |z|={rows[lv]['z']:.2f} "
                  f"{'PASS' if rows[lv]['z'] <= PAIRED_DIVERGENCE_Z_THRESHOLD else 'FAIL'}"
                  for lv in powered_levels))
    else:
        worst = max(order, key=lambda lv: rows[lv]["b"] + rows[lv]["c"])
        worst_bc = rows[worst]["b"] + rows[worst]["c"]
        print(f"paired gate: UNDERPOWERED at every level in this grid -- the largest "
              f"b + c is {worst_bc} at {level_name(worst)}, against "
              f"MINIMUM_USABLE_B_PLUS_C = {MINIMUM_USABLE_B_PLUS_C}.")
        # What the shortfall costs, in the only unit that can close it on this
        # population: more problems. Reported so the gap is a number rather
        # than the word "underpowered".
        if worst_bc > 0 and both:
            need = MINIMUM_USABLE_B_PLUS_C * len(both) / worst_bc
            print(f"             at that level's disagreement rate ({worst_bc}/{len(both)} "
                  f"of the problems both sides solved), reaching b + c = "
                  f"{MINIMUM_USABLE_B_PLUS_C} needs about {need:,.0f} solved-by-both "
                  f"problems.")
    return 0 if verdict == "MET" else 1


if __name__ == "__main__":
    sys.exit(main())
