#!/usr/bin/env python3
"""Re-derives every figure PORTING-PLAN.md's STOMP second-seed-base section
publishes.

Nothing is transcribed except where this script says so and prints the
comparison. Each number is computed from the NDJSON committed beside this
script -- the two port and two C++ arms at planner seed base 424242, the two
C++ arms at 700001 -- plus two files from `doc/phase8-condition2-stomp/`: the
port arm at 700001, and the seed-validity split (a property of the problem,
not of a planner seed, so it is not duplicated here).

Run with no argument to read them from this script's own directory and print
the report; pass `--check FILE` to compare the whole report against a
committed transcript instead (what
`tools/ci/check-phase8-seedbase-stomp.sh` runs) -- a number that drifts fails
by name rather than by a total, the same `--check`
`doc/phase8-baseline-500/rederive.py` uses.

# Condition-2 validity is not monotone in the resolution

`densify` steps a segment with `steps = ceil(distance / resolution)` and
samples at `i / steps`, so a finer resolution's sample set does NOT contain a
coarser one's -- for a 0.15-long segment, 0.02 samples at `i/8` and 0.01 at
`i/15`, sharing only the endpoints. A path can therefore fail at 0.02 and pass
at 0.01 because the denser grid steps over the thin invalid pocket the coarser
one landed in. `cage/161` at seed base 424242 does exactly that.

Two consequences this script is built around:

  * per-level counts are NOT cumulative, and the union of the levels' failing
    ids is NOT a set any single resolution produces. Every table below is
    per-level; the union is printed once, labelled as a union.
  * `r*` -- the finest level at which the C++ arm is 100% -- is still
    well-defined, but it is not a threshold. "100% at r*" does not imply
    "100% at every coarser level", and this script prints the whole column so
    a reader can see which it is.
"""
import argparse
import difflib
import io
import json
import sys
from pathlib import Path

CONFIGS = ("floor_wall", "cage")
# The grid both 424242 sides walked, coarse first. `returned` is no
# densification at all: the waypoints the planner actually handed back.
GRID = (0.2, 0.1, 0.05, 0.02, 0.01, 0.005, 0.002, 0.001)
RETURNED = "returned"
LEVELS = (RETURNED,) + GRID
# The levels the committed port@700001 arm can answer: it was swept with the
# grid `[0.05]`, and carries the native 0.01 verdict in `condition2_valid` and
# the undensified one in `condition2_valid_at_returned_waypoints`.
LEVELS_700001_PORT = (RETURNED, 0.05, 0.01)
# `tools/moveit-diff/src/main.rs`'s own constant, not a second opinion.
MINIMUM_USABLE_B_PLUS_C = 61

# PORTING-PLAN.md §286.3's one claim about the port@700001 arm that the
# committed file cannot answer, because that sweep's grid stopped at 0.05:
# below the native bar the port adds `cage/98`. Printed as a citation, never
# folded into a measured set.
CITED_286_3 = "port@700001 adds cage/98 at 0.005 and finer (§286.3; not in this data)"


def load(path):
    records = {}
    for line in Path(path).read_text().splitlines():
        if line.strip():
            record = json.loads(line)
            records[int(record["id"])] = record
    return records


def verdicts(record):
    """Every condition-2 verdict a solved record carries, keyed by level.

    None for an unsolved problem: it produced no path, so it has no condition-2
    answer, and counting it either way would be a statement about the solve
    rate wearing condition 2's name.
    """
    if not record.get("solved"):
        return None
    out = {RETURNED: bool(record["condition2_valid_at_returned_waypoints"])}
    for entry in record.get("condition2_by_resolution") or []:
        out[float(entry["resolution"])] = bool(entry["valid"])
    # The set's own `motion_resolution`. Both sweeps here put 0.01 in the grid
    # too; when they agree this is a free cross-check, and when a sweep has no
    # 0.01 grid entry (the port@700001 arm) it is the only source.
    native = bool(record["condition2_valid"])
    if 0.01 in out and out[0.01] != native:
        raise SystemExit(
            f"id {record['id']}: condition2_valid={native} but the grid's 0.01 "
            f"entry says {out[0.01]}; the two must agree"
        )
    out[0.01] = native
    return out


def read_arm(prefix, directory):
    """One arm over both configs, tagged `config/id` so ids cannot collide."""
    arm = {}
    for config in CONFIGS:
        for problem_id, record in load(directory / f"{prefix}.{config}.ndjson").items():
            arm[(config, problem_id)] = record
    return arm


def solved_of(arm):
    return {tag for tag, record in arm.items() if record.get("solved")}


def fails(arm, level, population=None):
    out = set()
    for tag, record in arm.items():
        if population is not None and tag not in population:
            continue
        v = verdicts(record)
        if v is not None and not v[level]:
            out.add(tag)
    return out


def name(level):
    return RETURNED if level == RETURNED else f"{level:g}"


def fmt(tags):
    return " ".join(f"{c}/{i}" for c, i in sorted(tags)) if tags else "(none)"


def report(directory, out):
    def say(line=""):
        print(line, file=out)

    base_dir = directory.parent / "phase8-condition2-stomp"

    seed = {}
    for config in CONFIGS:
        for problem_id, record in load(base_dir / f"seed.{config}.ndjson").items():
            seed[(config, problem_id)] = record
    seed_invalid = {tag for tag, record in seed.items() if not record["seed_valid"]}

    arms = {
        "port@424242": (read_arm("port.stomp", directory), LEVELS),
        "cpp@424242": (read_arm("stomp", directory), LEVELS),
        "port@700001": (read_arm("base.port.stomp", base_dir), LEVELS_700001_PORT),
        "cpp@700001": (read_arm("cpp700001", directory), LEVELS),
    }

    say("=" * 78)
    say("A. POPULATION AND SOLVE RATE")
    say("   n=500: floor_wall 250 @ set seed 900001, cage 250 @ 900002, panda_arm")
    say("=" * 78)
    say(f"  seed-invalid stratum: {len(seed_invalid)}/{len(seed)}")
    for label, (arm, _) in arms.items():
        s = solved_of(arm)
        si = s & seed_invalid
        say(f"  {label:12} solved {len(s):3}/{len(arm)}   "
            f"seed-valid {len(s) - len(si):3}/{len(seed) - len(seed_invalid)}   "
            f"seed-invalid {len(si):3}/{len(seed_invalid)}")

    say()
    say("=" * 78)
    say("B. CONDITION-2 FAILURES PER LEVEL -- counts are NOT cumulative")
    say("   (see this script's docstring: validity is not monotone in resolution)")
    say("=" * 78)
    say("  " + f"{'level':>10}" + "".join(f"{n:>17}" for n in arms))
    for level in LEVELS:
        cells = ""
        for label, (arm, own) in arms.items():
            if level not in own:
                cells += f"{'--':>17}"
            else:
                cells += f"{len(fails(arm, level)):>11} /{len(solved_of(arm)):<5}"
        say(f"  {name(level):>10}{cells}")
    say()
    say("  seed-invalid stratum only:")
    say("  " + f"{'level':>10}" + "".join(f"{n:>17}" for n in arms))
    for level in LEVELS:
        cells = ""
        for label, (arm, own) in arms.items():
            if level not in own:
                cells += f"{'--':>17}"
            else:
                si_solved = solved_of(arm) & seed_invalid
                cells += f"{len(fails(arm, level, si_solved)):>11} /{len(si_solved):<5}"
        say(f"  {name(level):>10}{cells}")

    say()
    say("=" * 78)
    say("C. THE FAILING IDS AT EVERY LEVEL, PER ARM")
    say("   The last line of each arm is the UNION over levels. It is NOT a set")
    say("   any single resolution produces -- do not quote it as one.")
    say("=" * 78)
    for label, (arm, own) in arms.items():
        say(f"  {label}:")
        union = set()
        for level in LEVELS:
            if level not in own:
                say(f"    {name(level):>10}  (not in this arm's grid)")
                continue
            f = fails(arm, level)
            union |= f
            say(f"    {name(level):>10}  {fmt(f)}")
        say(f"    {'UNION':>10}  {fmt(union)}   <- union over levels, not a resolution")
        if label == "port@700001":
            say(f"    {'':>10}  {CITED_286_3}")

    say()
    say("=" * 78)
    say("D. RETURNED-WAYPOINT VERDICT AND SEED VALIDITY FOR EVERY FAILING PROBLEM")
    say("=" * 78)
    for label, (arm, own) in arms.items():
        union = set()
        for level in own:
            union |= fails(arm, level)
        say(f"  {label}: {len(union)} problem(s) fail at one or more levels")
        for tag in sorted(union):
            v = verdicts(arm[tag])
            bad = [name(lv) for lv in own if not v[lv]]
            good = [name(lv) for lv in own if v[lv]]
            say(f"    {tag[0]}/{tag[1]:<4} seed={424242 if '424242' in label else 700001}"
                f"+{tag[1]}  fails {' '.join(bad)}  passes {' '.join(good)}")
            say(f"    {'':<11} returned-waypoints valid = {v[RETURNED]}"
                f"   seed_valid = {seed[tag]['seed_valid']}")

    say()
    say("=" * 78)
    say("E. INTERSECTION BETWEEN THE TWO SEED BASES, PER LEVEL")
    say("   Every row compares two sets measured at the SAME level.")
    say("=" * 78)
    for who, a_label, b_label in (("port", "port@424242", "port@700001"),
                                  ("cpp ", "cpp@424242", "cpp@700001")):
        a_arm, a_own = arms[a_label]
        b_arm, b_own = arms[b_label]
        say(f"  {who}: {a_label} vs {b_label}")
        for level in LEVELS:
            if level not in a_own or level not in b_own:
                continue
            a, b = fails(a_arm, level), fails(b_arm, level)
            say(f"    {name(level):>10}  424242={fmt(a):<40} 700001={fmt(b):<32} "
                f"intersection={fmt(a & b)}")

    say()
    say("  cross-implementation at the native bar 0.01:")
    for base in ("424242", "700001"):
        p = fails(arms[f"port@{base}"][0], 0.01)
        c = fails(arms[f"cpp@{base}"][0], 0.01)
        say(f"    @{base}  port={fmt(p):<34} cpp={fmt(c):<34} intersection={fmt(p & c)}")

    say()
    say("=" * 78)
    say("F. r* -- the finest level at which upstream C++ STOMP is 100%")
    say("   Not a threshold: validity is not monotone, so read the column in B.")
    say("=" * 78)
    for label in ("cpp@700001", "cpp@424242"):
        arm, own = arms[label]
        solved = solved_of(arm)
        perfect = [lv for lv in LEVELS if lv in own and not fails(arm, lv)]
        r_star = perfect[-1] if perfect else None
        say(f"  {label}: r* = {name(r_star) if r_star is not None else 'NONE'}"
            f"   100% over its own {len(solved)} solved")
        say(f"      levels at 100%: {' '.join(name(lv) for lv in perfect) or '(none)'}")
        if r_star is not None:
            port_arm = arms["port@424242"][0]
            port_solved = solved_of(port_arm)
            bad = fails(port_arm, r_star)
            say(f"      port@424242 at that level: "
                f"{len(port_solved) - len(bad)}/{len(port_solved)} -> "
                f"{'MET' if not bad else 'UNMET ' + fmt(bad)}")

    say()
    say("=" * 78)
    say("G. PAIRED TABLE AT 424242 (b = port fails where cpp passes, c = reverse)")
    say("=" * 78)
    port_arm = arms["port@424242"][0]
    cpp_arm = arms["cpp@424242"][0]
    both = solved_of(port_arm) & solved_of(cpp_arm)
    for population, pname in ((both, f"both solved, n={len(both)}"),
                              (both & seed_invalid,
                               f"both solved & seed-invalid, n={len(both & seed_invalid)}")):
        say(f"  {pname}")
        for level in LEVELS:
            pf = fails(port_arm, level, population)
            cf = fails(cpp_arm, level, population)
            b, c = len(pf - cf), len(cf - pf)
            flag = "powered" if b + c >= MINIMUM_USABLE_B_PLUS_C else "UNDERPOWERED"
            say(f"    {name(level):>10}  b={b:<3} c={c:<3} b+c={b + c:<3} {flag}")

    say()
    say("=" * 78)
    say("H. NON-MONOTONE CASES -- a problem that fails at a level and passes at a")
    say("   FINER one. Enumerated because the union in C is meaningless without")
    say("   them, and because a reader will otherwise assume a threshold.")
    say("=" * 78)
    found = 0
    for label, (arm, own) in arms.items():
        ordered = [lv for lv in GRID if lv in own]
        for tag in sorted(solved_of(arm)):
            v = verdicts(arm[tag])
            for i, coarse in enumerate(ordered):
                if v[coarse]:
                    continue
                finer_pass = [f for f in ordered[i + 1:] if v[f]]
                if finer_pass:
                    found += 1
                    say(f"  {label} {tag[0]}/{tag[1]}: fails at {name(coarse)}, "
                        f"passes at finer {' '.join(name(f) for f in finer_pass)}")
                    break
    say(f"  total: {found}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--dir", default=str(Path(__file__).resolve().parent))
    parser.add_argument("--check")
    args = parser.parse_args()

    buffer = io.StringIO()
    report(Path(args.dir), buffer)
    text = buffer.getvalue()

    if args.check:
        want = Path(args.check).read_text()
        if want != text:
            sys.stdout.write(text)
            print(f"FAIL {args.check} does not match this run", file=sys.stderr)
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
