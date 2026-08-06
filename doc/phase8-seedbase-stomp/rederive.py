#!/usr/bin/env python3
"""Re-derives every figure PORTING-PLAN.md's STOMP second-seed-base section
publishes.

Nothing is transcribed. Each number is computed from the NDJSON files
committed beside this script -- the four arms at planner seed base 424242, the
two upstream arms at 700001, and the two seed-validity files (which are the
same population's, taken from `doc/phase8-condition2-stomp/` because seed
validity is a property of the problem, not of a planner seed).

Run with no argument to read this script's own directory.
"""
import json
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
OUT = Path(sys.argv[1]) if len(sys.argv) > 1 else HERE
SEED_DIR = HERE.parent / "phase8-condition2-stomp"

CONFIGS = ("floor_wall", "cage")
# The grid both sides walked, coarse first. `returned` is no densification at
# all: the waypoints the planner actually handed back.
GRID = (0.2, 0.1, 0.05, 0.02, 0.01, 0.005, 0.002, 0.001)
RETURNED = "returned"
# `tools/moveit-diff/src/main.rs`'s own constants, not a second opinion.
MINIMUM_USABLE_B_PLUS_C = 61

# PORTING-PLAN.md §286.3's published id sets at seed base 700001, transcribed
# here ONLY so this script can print the intersection against them. The cpp
# side of that column is also re-measured below from `cpp700001.*.ndjson`, so
# a transcription error here shows up as a mismatch rather than propagating.
PUBLISHED_700001 = {
    "port": {("floor_wall", 77), ("cage", 98), ("cage", 133), ("cage", 159)},
    "cpp": {("floor_wall", 84), ("floor_wall", 120), ("floor_wall", 230)},
}


def load(path):
    records = {}
    for line in Path(path).read_text().splitlines():
        if line.strip():
            record = json.loads(line)
            records[int(record["id"])] = record
    return records


def verdicts(record):
    """Every condition-2 verdict a solved record carries, keyed by level.

    Returns None for an unsolved problem: it produced no path, so it has no
    condition-2 answer, and counting it as either a pass or a failure would be
    a statement about the solve rate wearing condition 2's name.
    """
    if not record.get("solved"):
        return None
    out = {RETURNED: bool(record["condition2_valid_at_returned_waypoints"])}
    for entry in record["condition2_by_resolution"]:
        out[float(entry["resolution"])] = bool(entry["valid"])
    return out


def read_arm(prefix, directory=OUT):
    """One arm over both configs, tagged `config/id` so ids cannot collide."""
    arm = {}
    for config in CONFIGS:
        for problem_id, record in load(directory / f"{prefix}.{config}.ndjson").items():
            arm[(config, problem_id)] = record
    return arm


def fails(arm, level, population=None):
    out = set()
    for tag, record in arm.items():
        if population is not None and tag not in population:
            continue
        v = verdicts(record)
        if v is not None and not v[level]:
            out.add(tag)
    return out


def solved_of(arm):
    return {tag for tag, record in arm.items() if record.get("solved")}


def fmt(tags):
    if not tags:
        return "(none)"
    return " ".join(f"{c}/{i}" for c, i in sorted(tags))


seed = {}
for config in CONFIGS:
    for problem_id, record in load(SEED_DIR / f"seed.{config}.ndjson").items():
        seed[(config, problem_id)] = record
seed_invalid = {tag for tag, record in seed.items() if not record["seed_valid"]}

arms = {
    "port@424242": read_arm("port.stomp"),
    "cpp@424242": read_arm("stomp"),
    "cpp@700001": read_arm("cpp700001"),
}
levels = [RETURNED] + list(GRID)

print("=" * 78)
print("A. POPULATION AND SOLVE RATE  (n=500: floor_wall 250 @ 900001, cage 250 @ 900002)")
print("=" * 78)
print(f"  seed-invalid stratum: {len(seed_invalid)}/{len(seed)}")
for name, arm in arms.items():
    s = solved_of(arm)
    si = s & seed_invalid
    print(f"  {name:12} solved {len(s):3}/{len(arm)}   "
          f"seed-valid {len(s) - len(si):3}/{len(seed) - len(seed_invalid)}   "
          f"seed-invalid {len(si):3}/{len(seed_invalid)}")

print()
print("=" * 78)
print("B. CONDITION-2 FAILURES PER LEVEL  (cell = failing problems / that arm's solved)")
print("=" * 78)
header = "  " + f"{'level':>10}" + "".join(f"{n:>16}" for n in arms)
print(header)
for level in levels:
    label = RETURNED if level == RETURNED else f"{level:g}"
    cells = ""
    for name, arm in arms.items():
        cells += f"{len(fails(arm, level)):>10} /{len(solved_of(arm)):<5}"
    print(f"  {label:>10}{cells}")

print()
print("  seed-invalid stratum only:")
print(header)
for level in levels:
    label = RETURNED if level == RETURNED else f"{level:g}"
    cells = ""
    for name, arm in arms.items():
        si_solved = solved_of(arm) & seed_invalid
        cells += f"{len(fails(arm, level, si_solved)):>10} /{len(si_solved):<5}"
    print(f"  {label:>10}{cells}")

print()
print("=" * 78)
print("C. EVERY FAILING ID, AND WHETHER ITS OWN RETURNED WAYPOINTS PASS")
print("=" * 78)
for name, arm in arms.items():
    union = set()
    for level in levels:
        union |= fails(arm, level)
    print(f"  {name}: {len(union)} problem(s) fail somewhere in the grid")
    for tag in sorted(union):
        v = verdicts(arm[tag])
        bad = [RETURNED if lv == RETURNED else f"{lv:g}" for lv in levels if not v[lv]]
        print(f"    {tag[0]}/{tag[1]:<4} fails at: {' '.join(bad)}"
              f"   | returned-waypoints valid = {v[RETURNED]}"
              f"   | seed_valid = {seed[tag]['seed_valid']}")

print()
print("=" * 78)
print("D. INTERSECTION WITH SEED BASE 700001")
print("=" * 78)


def union_fails(arm):
    out = set()
    for level in levels:
        out |= fails(arm, level)
    return out


port_424242 = union_fails(arms["port@424242"])
cpp_424242 = union_fails(arms["cpp@424242"])
cpp_700001 = union_fails(arms["cpp@700001"])
print(f"  port @424242 : {fmt(port_424242)}")
print(f"  cpp  @424242 : {fmt(cpp_424242)}")
print(f"  cpp  @700001 (re-measured here) : {fmt(cpp_700001)}")
print(f"  cpp  @700001 (§286.3 as published) : {fmt(PUBLISHED_700001['cpp'])}")
agree = cpp_700001 == PUBLISHED_700001["cpp"]
print(f"  re-measured cpp@700001 == §286.3's set: {agree}")
print(f"  port @700001 (§286.3 as published) : {fmt(PUBLISHED_700001['port'])}")
print()
for label, a, b in (
    ("port@424242 vs port@700001", port_424242, PUBLISHED_700001["port"]),
    ("cpp@424242  vs cpp@700001 ", cpp_424242, cpp_700001),
    ("port@424242 vs cpp@424242 ", port_424242, cpp_424242),
    ("port@700001 vs cpp@700001 ", PUBLISHED_700001["port"], cpp_700001),
):
    print(f"  {label}: intersection {fmt(a & b)}  (|A|={len(a)} |B|={len(b)})")

print()
print("=" * 78)
print("E. r* -- the finest level at which upstream C++ STOMP is 100%")
print("=" * 78)
for name in ("cpp@700001", "cpp@424242"):
    arm = arms[name]
    solved = solved_of(arm)
    perfect = [lv for lv in levels if not fails(arm, lv)]
    r_star = perfect[-1] if perfect else None
    label = "NONE" if r_star is None else (RETURNED if r_star == RETURNED else f"{r_star:g}")
    print(f"  {name}: r* = {label}   (100% over its own {len(solved)} solved)")
    if r_star is not None and levels.index(r_star) + 1 < len(levels):
        nxt = levels[levels.index(r_star) + 1]
        print(f"      one level finer ({nxt:g}): "
              f"{len(solved) - len(fails(arm, nxt))}/{len(solved)}")
    if r_star is not None:
        port = arms["port@424242"]
        port_solved = solved_of(port)
        bad = fails(port, r_star)
        print(f"      port@424242 at that level: "
              f"{len(port_solved) - len(bad)}/{len(port_solved)} -> "
              f"{'MET' if not bad else 'UNMET ' + fmt(bad)}")

print()
print("=" * 78)
print("F. PAIRED TABLE AT 424242  (b = port fails where cpp passes, c = reverse)")
print("=" * 78)
port, cpp = arms["port@424242"], arms["cpp@424242"]
both = solved_of(port) & solved_of(cpp)
both_si = both & seed_invalid
for population, pname in ((both, f"both solved, n={len(both)}"),
                          (both_si, f"both solved & seed-invalid, n={len(both_si)}")):
    print(f"  {pname}")
    for level in levels:
        label = RETURNED if level == RETURNED else f"{level:g}"
        pf = fails(port, level, population)
        cf = fails(cpp, level, population)
        b, c = len(pf - cf), len(cf - pf)
        flag = "powered" if b + c >= MINIMUM_USABLE_B_PLUS_C else "UNDERPOWERED"
        print(f"    {label:>10}  b={b:<3} c={c:<3} b+c={b + c:<3} {flag}")
