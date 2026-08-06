#!/usr/bin/env python3
"""Re-derives every figure §300 will publish, from the four sweep NDJSON files.

Nothing here is transcribed: each number is computed from base/ and mut/
port.stomp.<config>.ndjson and seed/seed.<config>.ndjson.
"""
import json
import sys
from pathlib import Path

OUT = Path(sys.argv[1])
CONFIGS = ("floor_wall", "cage")
BAR = 0.05


def load(p):
    out = {}
    for line in Path(p).read_text().splitlines():
        if line.strip():
            r = json.loads(line)
            out[int(r["id"])] = r
    return out


def bar_ok(r):
    return all(bool(e["valid"]) for e in r["condition2_by_resolution"]
               if float(e["resolution"]) == BAR)


def bar_invalid(r):
    return sum(e["invalid_count"] for e in r["condition2_by_resolution"]
               if float(e["resolution"]) == BAR)


seed = {c: load(OUT / "seed" / f"seed.{c}.ndjson") for c in CONFIGS}
base = {c: load(OUT / "base" / f"port.stomp.{c}.ndjson") for c in CONFIGS}
mut = {c: load(OUT / "mut" / f"port.stomp.{c}.ndjson") for c in CONFIGS}

print("=" * 72)
print("A. SOLVE RATE (the mutation's own signature)")
print("=" * 72)
for c in CONFIGS:
    b = sum(1 for r in base[c].values() if r.get("solved"))
    m = sum(1 for r in mut[c].values() if r.get("solved"))
    print(f"  {c:11} base {b}/250   mut {m}/250")

print()
print("=" * 72)
print("B. PAIRED ON BASE-SOLVED IDS -- the only non-confounded comparison")
print("=" * 72)
paired_missed = []
for c in CONFIGS:
    ids = [i for i in sorted(base[c]) if base[c][i].get("solved")]
    row = {}
    for label, arm in (("base", base), ("mut", mut)):
        row[label] = dict(
            native=sum(1 for i in ids if not arm[c][i]["condition2_valid"]),
            bar=sum(1 for i in ids if not bar_ok(arm[c][i])),
            ret=sum(1 for i in ids
                    if not arm[c][i]["condition2_valid_at_returned_waypoints"]),
        )
    print(f"  {c} (n={len(ids)} base-solved)")
    for k, name in (("native", "native motion_resolution 0.01"),
                    ("bar", "0.05 grid bar"),
                    ("ret", "returned waypoints")):
        print(f"    {name:32} base {row['base'][k]:4} -> mut {row['mut'][k]:4}")
    # ids native catches that the 0.05 grid calls clean, on the base arm
    for i in ids:
        r = base[c][i]
        if not r["condition2_valid"] and bar_ok(r):
            paired_missed.append((c, i, r["invalid_waypoint_count"],
                                  r["condition2_by_resolution"]))

print()
print("=" * 72)
print("C. DOMINANCE: base-arm ids the native bar catches and 0.05 calls clean")
print("=" * 72)
for c, i, iw, grid in paired_missed:
    g = grid[0]
    print(f"  {c:11} id {i:3}  native invalid_wp={iw}  ->  0.05 densifies to "
          f"{g['densified_waypoint_count']} wp, invalid_count={g['invalid_count']}, "
          f"valid={g['valid']}")
print(f"  total native-only violations: {len(paired_missed)}")
rev = []
for c in CONFIGS:
    for i in sorted(base[c]):
        r = base[c][i]
        if r.get("solved") and r["condition2_valid"] and not bar_ok(r):
            rev.append((c, i))
print(f"  reverse direction (0.05 catches, native clean): {len(rev)}  {rev}")

print()
print("=" * 72)
print("D. WHAT THE MUTATION ARM ACTUALLY FLAGS vs the seed's own invalidity")
print("=" * 72)
for c in CONFIGS:
    seed_bad = {i for i in seed[c] if not seed[c][i]["seed_valid"]}
    mut_bad = {i for i in mut[c] if mut[c][i].get("solved") and not bar_ok(mut[c][i])}
    print(f"  {c:11} seed-invalid {len(seed_bad):3}   mut 0.05-invalid {len(mut_bad):3}   "
          f"overlap {len(seed_bad & mut_bad):3}   mut-only {len(mut_bad - seed_bad):3}   "
          f"seed-only {len(seed_bad - mut_bad):3}")
    if mut_bad - seed_bad:
        print(f"    mut-only ids: {sorted(mut_bad - seed_bad)}")
    if seed_bad - mut_bad:
        for i in sorted(seed_bad - mut_bad):
            m = mut[c][i]
            print(f"    seed-only id {i}: seed_invalid_count="
                  f"{seed[c][i]['seed_invalid_count']} seed_length={seed[c][i]['seed_length']!r}"
                  f" | mut length={m.get('length')!r} bar_ok={bar_ok(m)}"
                  f" native_ok={m['condition2_valid']}")

print()
print("=" * 72)
print("E. NO-OP RATE: path length bit-identical to the seed's length")
print("=" * 72)
for label, arm in (("base", base), ("mut", mut)):
    for c in CONFIGS:
        soln = [i for i in sorted(arm[c]) if arm[c][i].get("solved")]
        same = [i for i in soln
                if arm[c][i].get("length") == seed[c][i].get("seed_length")]
        pct = 100.0 * len(same) / len(soln) if soln else 0.0
        print(f"  {label:4} {c:11} length == seed_length on {len(same):3}/{len(soln):3} "
              f"solved ({pct:.1f}%)")
