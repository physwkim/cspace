#!/usr/bin/env python3
"""Re-derives every figure the STOMP condition-2 plan section publishes.

Nothing is transcribed: each number is computed from the four sweep NDJSON
files and the two seed-validity files committed beside this script. Run it
with no argument to read them from this script's own directory and print the
report; pass `--check FILE` to compare the whole report against a committed
transcript instead (what `tools/ci/check-phase8-condition2-stomp.sh` runs) --
a number that drifts fails by name rather than by a total, the same
`--check` `doc/phase8-baseline-500/rederive.py` uses.
"""
import argparse
import difflib
import io
import json
import sys
from pathlib import Path

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


def report(directory, out):
    def say(line=""):
        print(line, file=out)

    seed = {c: load(directory / f"seed.{c}.ndjson") for c in CONFIGS}
    base = {c: load(directory / f"base.port.stomp.{c}.ndjson") for c in CONFIGS}
    mut = {c: load(directory / f"mut.port.stomp.{c}.ndjson") for c in CONFIGS}

    say("=" * 72)
    say("A. SOLVE RATE (the mutation's own signature)")
    say("=" * 72)
    for c in CONFIGS:
        b = sum(1 for r in base[c].values() if r.get("solved"))
        m = sum(1 for r in mut[c].values() if r.get("solved"))
        say(f"  {c:11} base {b}/250   mut {m}/250")

    say()
    say("=" * 72)
    say("B. PAIRED ON BASE-SOLVED IDS -- the only non-confounded comparison")
    say("=" * 72)
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
        say(f"  {c} (n={len(ids)} base-solved)")
        for k, name in (("native", "native motion_resolution 0.01"),
                        ("bar", "0.05 grid bar"),
                        ("ret", "returned waypoints")):
            say(f"    {name:32} base {row['base'][k]:4} -> mut {row['mut'][k]:4}")
        # ids native catches that the 0.05 grid calls clean, on the base arm
        for i in ids:
            r = base[c][i]
            if not r["condition2_valid"] and bar_ok(r):
                paired_missed.append((c, i, r["invalid_waypoint_count"],
                                      r["condition2_by_resolution"]))

    say()
    say("=" * 72)
    say("C. base-arm ids the native bar catches and 0.05 calls clean.")
    say("   NOT a dominance result: PORTING-PLAN.md's §286.5 measured that a")
    say("   100% bar finer than the planner's own unit is a coin flip for BOTH")
    say("   implementations, so these are lottery candidates until an upstream")
    say("   column exists to compare them against.")
    say("=" * 72)
    for c, i, iw, grid in paired_missed:
        g = grid[0]
        say(f"  {c:11} id {i:3}  native invalid_wp={iw}  ->  0.05 densifies to "
            f"{g['densified_waypoint_count']} wp, invalid_count={g['invalid_count']}, "
            f"valid={g['valid']}")
    say(f"  total native-only violations: {len(paired_missed)}")
    rev = []
    for c in CONFIGS:
        for i in sorted(base[c]):
            r = base[c][i]
            if r.get("solved") and r["condition2_valid"] and not bar_ok(r):
                rev.append((c, i))
    say(f"  reverse direction (0.05 catches, native clean): {len(rev)}  {rev}")

    say()
    say("=" * 72)
    say("D. WHAT THE MUTATION ARM ACTUALLY FLAGS vs the seed's own invalidity")
    say("=" * 72)
    for c in CONFIGS:
        seed_bad = {i for i in seed[c] if not seed[c][i]["seed_valid"]}
        mut_bad = {i for i in mut[c] if mut[c][i].get("solved") and not bar_ok(mut[c][i])}
        say(f"  {c:11} seed-invalid {len(seed_bad):3}   mut 0.05-invalid {len(mut_bad):3}   "
            f"overlap {len(seed_bad & mut_bad):3}   mut-only {len(mut_bad - seed_bad):3}   "
            f"seed-only {len(seed_bad - mut_bad):3}")
        if mut_bad - seed_bad:
            say(f"    mut-only ids: {sorted(mut_bad - seed_bad)}")
        if seed_bad - mut_bad:
            for i in sorted(seed_bad - mut_bad):
                m = mut[c][i]
                say(f"    seed-only id {i}: seed_invalid_count="
                    f"{seed[c][i]['seed_invalid_count']} seed_length={seed[c][i]['seed_length']!r}"
                    f" | mut length={m.get('length')!r} bar_ok={bar_ok(m)}"
                    f" native_ok={m['condition2_valid']}")

    say()
    say("=" * 72)
    say("E. NO-OP RATE: path length equal to the seed's length.")
    say("   Exact float equality and tolerance equality are different numbers")
    say("   and both are published. The tolerance column is identical from")
    say("   1e-12 to 1e-6, so it is a real gap in the distribution rather than")
    say("   a threshold chosen to produce it. Length equality is NOT trajectory")
    say("   equality -- the harness emits no waypoint matrix to check that.")
    say("=" * 72)
    for label, arm in (("base", base), ("mut", mut)):
        for c in CONFIGS:
            soln = [i for i in sorted(arm[c]) if arm[c][i].get("solved")]
            exact = [i for i in soln
                     if arm[c][i].get("length") == seed[c][i].get("seed_length")]
            near = [i for i in soln
                    if abs(arm[c][i]["length"] - seed[c][i]["seed_length"]) < 1e-9]
            ep = 100.0 * len(exact) / len(soln) if soln else 0.0
            np_ = 100.0 * len(near) / len(soln) if soln else 0.0
            say(f"  {label:4} {c:11} exact {len(exact):3}/{len(soln):3} ({ep:4.1f}%)"
                f"   within 1e-9 {len(near):3}/{len(soln):3} ({np_:4.1f}%)")
    say()
    say("  tolerance sweep (shows the plateau):")
    for tol in (1e-12, 1e-9, 1e-6, 1e-3):
        cells = []
        for label, arm in (("base", base), ("mut", mut)):
            for c in CONFIGS:
                soln = [i for i in sorted(arm[c]) if arm[c][i].get("solved")]
                n = sum(1 for i in soln
                        if abs(arm[c][i]["length"] - seed[c][i]["seed_length"]) < tol)
                cells.append(f"{n}/{len(soln)}")
        say(f"    {tol:<8g} base {cells[0]:>8} {cells[1]:>8}   mut {cells[2]:>8} {cells[3]:>8}")


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
