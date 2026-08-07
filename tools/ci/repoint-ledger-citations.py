#!/usr/bin/env python3
"""Re-derive the assertion-ledger `file.rs:NNN` citations a merge moved.

The sibling of `repoint-in-repo-citations.py` for the other citation
population: the first column of every `doc/assertion-discrimination-ledger-*.md`
row. A merge that adds tests to a cited `.rs` file shifts every assertion
below the insertion, and each ledger row then names a line its assertion no
longer occupies. That is not hypothetical -- it is how
`verify-orphan-enumeration.sh` went red on main twice, the second time with
11 orphan sites and 8 unresolved citations in one file pair.

Two rules, the same two that make the in-repo repointer safe to run:

1. **The gate decides what is broken, not this script.** It calls
   `reconcile-assertion-ledgers.py`'s own `reconcile()` and repairs exactly
   the citations that run puts in `unresolved`. A citation the gate resolves
   is left alone.

2. **Relocation is by content, never by arithmetic.** A row moves only to a
   scanner site inside the test function THE ROW ITSELF NAMES in its third
   column, and only when that choice is unique. Nearest-line proximity is
   not used and must not be: it picked `collision_parity.rs:1633` -- an
   assertion in `pr2_world_object_pair_flip_case_122_both_sides_are_real_
   vertices` -- for a row whose own column 3 reads
   `pr2_self_wheel_same_pair_oracle_magnitude_is_implausible`, so the row
   vouched for an assertion nobody had measured while the one it did
   measure sat in the orphan list.

Which part of a row names that function differs per ledger -- p3-acm puts it
in column 3, pilz in column 4 (column 3 there names the production function
under test), p9-ros inside a prose cell as ``:914 (`the_test_name`)``. So
the key is not hardcoded but LEARNED per ledger, by validating candidates
against that ledger's own citations that resolve exactly today. That
learning now lives in the gate (`learn_keys`, `subject_mismatch` in
reconcile-assertion-ledgers.py), because the gate is what has to REJECT a
citation resolving into a test its row does not name; this tool imports it
and reuses the same keys to decide where such a row belongs.

The 100%-agreement bar there is not decoration. Widening the search to "any
identifier anywhere in the row that names a function holding scanner sites"
raises coverage from 272 to 433 of the 1087 exact citations -- and mispoints
8, every one of them onto a SIBLING test the row's prose mentions ("same
reasoning as ...") rather than the row's own. Coverage bought that way is how
a ledger ends up vouching for an assertion nobody measured.

Among the sites inside the named function, only ORPHAN sites are candidates:
a site another row already accounts for must not be stolen by this one. Of
those, in order: a site in the FIRST population wins over one that exists
only because `half_plane`/`cmp_compound` were added (§307) -- a row that
predates those kinds cannot have measured a site the scanner did not then
emit; then a site whose scanner kind equals the row's own kind column; then
a lone remaining orphan. Anything else is reported and skipped. The kind
tie-break is deliberately last and best-effort: column 2 is not a uniform
field across the ledgers (`bare`, `yes`, `same`, `**no**` and a ledger-name
list all occur there), so it can confirm a choice but must never be the only
thing making one.

    tools/ci/repoint-ledger-citations.py [--apply]

Without --apply it prints the mapping and changes nothing.

This is a repair tool, not a gate -- deliberately not named `check-*` or
`verify-*`, so the CI glob does not run it.
"""
import importlib.util
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
RECONCILE = ROOT / "tools/ci/reconcile-assertion-ledgers.py"


def load_reconcile():
    spec = importlib.util.spec_from_file_location("reconcile", RECONCILE)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def row_kind(mod, raw_row):
    cells = mod.row_cells(raw_row)
    return cells[2].strip() if len(cells) >= 3 else ""


def candidate_files(mod, fname_part, sites):
    paths = {p for (p, _) in sites}
    if "/" in fname_part:
        return sorted(p for p in paths if mod.path_matches(p, fname_part))
    return sorted(p for p in paths if p.rsplit("/", 1)[-1] == fname_part)


def plan(mod):
    """[(ledger, fname_part, old_line, new_line)] plus [(citation, why)] for
    every unresolved citation this tool declines to move."""
    result = mod.reconcile()
    sites = result["sites"]
    keys = mod.learn_keys(sites, result["basenames"], result["spans"])
    for ledger, ks in sorted(keys.items()):
        print(f"  key {ledger}: {', '.join(k.name for k in ks) or '(none validated)'}")
    orphans = set(result["orphans"])
    first_population = set(result["orphans_first"])
    moves, skipped = [], []
    row_cache = {}
    # Pass 1: give every unresolved citation its (source file, subject fn), or
    # a reason it has none. Pass 2 needs the whole set at once -- see below.
    grouped = {}
    for ledger, fname_part, lineno, _short, _status, _cat, _detail in result["unresolved"]:
        cite = f"{ledger} -> {fname_part}:{lineno}"
        if ledger not in row_cache:
            row_cache[ledger] = mod.full_rows(ledger)
        raw = row_cache[ledger].get((fname_part, lineno))
        if raw is None:
            skipped.append((cite, "no table row carries this citation"))
            continue
        name = next(
            (n for k in keys.get(ledger, []) if (n := k(raw, lineno)) is not None), None
        )
        if name is None:
            skipped.append((cite, "no validated key names a function in this row"))
            continue
        files = candidate_files(mod, fname_part, sites)
        if len(files) != 1:
            skipped.append((cite, f"{len(files)} source file(s) match this citation"))
            continue
        path = files[0]
        spans = mod.fn_span(path, name)
        if len(spans) != 1:
            skipped.append((cite, f"`fn {name}` is defined {len(spans)}x in {path}"))
            continue
        grouped.setdefault((path, name, spans[0]), []).append(
            (ledger, fname_part, lineno, raw)
        )

    # Pass 2: assign per subject function, not per citation. Two rows of one
    # ledger citing two assertions in the same test drift together, and a
    # citation-at-a-time loop sees "2 candidates" for each and gives up on
    # both. With the group in hand the N-rows/N-orphans case is settled by
    # order: an insertion above shifts every site in the function equally, so
    # the rows' relative order is preserved even when their line numbers are
    # not. That is a content fact about how drift happens, not a guess.
    for (path, name, (lo, hi)), rows in sorted(grouped.items()):
        inside = sorted(line for (p, line) in orphans if p == path and lo <= line <= hi)
        rows.sort(key=lambda r: r[2])
        cites = [f"{r[0]} -> {r[1]}:{r[2]}" for r in rows]
        if not inside:
            for c in cites:
                skipped.append((c, f"`fn {name}` holds no orphan scanner site"))
            continue
        if len(rows) == len(inside) and len(rows) > 1:
            chosen = inside
        elif len(rows) == 1:
            ledger, fname_part, lineno, raw = rows[0]
            first = [line for line in inside if (path, line) in first_population]
            want = row_kind(mod, raw)
            same_kind = [line for line in inside if sites[(path, line)] == want]
            if len(inside) == 1:
                chosen = inside
            elif len(first) == 1:
                chosen = first
            elif len(same_kind) == 1:
                chosen = same_kind
            else:
                skipped.append(
                    (cites[0], f"`fn {name}` holds {len(inside)} orphan sites {inside} "
                               f"({len(first)} in the first population, {len(same_kind)} "
                               f"of kind {want!r}) -- ambiguous")
                )
                continue
        else:
            for c in cites:
                skipped.append(
                    (c, f"`fn {name}` holds {len(inside)} orphan sites {inside} for "
                        f"{len(rows)} drifted row(s) -- ambiguous")
                )
            continue
        for (ledger, fname_part, lineno, _raw), new in zip(rows, chosen):
            orphans.discard((path, new))
            moves.append((ledger, fname_part, lineno, new, name))
    return moves, skipped


def cite_re(fname_part, old):
    """`<file>:<old>` as a whole citation token. The optional leading comma
    group keeps a multi-site row (`a.rs:4371,4292`) intact, and the trailing
    lookahead stops `a.rs:436` from matching inside `a.rs:4364` or eating
    half of a span (`a.rs:542-582`)."""
    return re.compile(
        re.escape(f"{fname_part}:") + r"((?:\d+\s*,\s*)*)" + str(old) + r"(?![\d-])"
    )


def adjacency_re(old):
    """``:914 (`the_test_name`)`` -- the gate's `adjacent_key` grammar, whose
    line number is NOT part of a `file.rs:NNN` citation and so survives
    cite_re() untouched. Left stale it silently disables that row's subject
    key, which is the one thing standing between the row and being credited
    for a neighbour's assertion."""
    return re.compile(
        r"(?<!\d):" + str(old) + r"(?![\d-])(`?\s*\(`[a-z_][a-z0-9_]{5,}`\))"
    )


def apply_moves(moves):
    """Rewrite each moved citation EVERYWHERE in its own ledger, not only in
    the row's first column. A ledger's prose cites the same site again --
    `parry.rs:4623` appears both as a row key and inside that ledger's
    "0 misverdicts" audit paragraph -- and those prose citations are in
    `check-citation-drift.py`'s corpus, so a first-column-only rewrite trades
    one gate's failure for another's. A document-wide rewrite is safe here
    precisely because the plan is 1:1: if two rows cited the same old line
    they would both be unresolved and could want different new lines, so
    that case refuses to write rather than pick one.

    The `:NNN (`fn`)` adjacency labels move too, but only on the lines the
    citation rewrite actually touched: that spelling carries no file name, so
    document-wide it would collide with any other row's line number."""
    by_ledger = {}
    for ledger, fname_part, old, new, _fn in moves:
        by_ledger.setdefault(ledger, []).append((fname_part, old, new))
    total = 0
    for ledger, items in by_ledger.items():
        seen = {}
        for fname_part, old, new in items:
            if seen.setdefault((fname_part, old), new) != new:
                raise SystemExit(
                    f"FAIL {ledger}: {fname_part}:{old} is cited by two rows that "
                    f"want different targets ({seen[(fname_part, old)]} and {new}) "
                    f"-- resolve those two rows by hand"
                )
        path = ROOT / ledger
        lines = original = path.read_text(encoding="utf-8").split("\n")
        for fname_part, old, new in items:
            pattern = cite_re(fname_part, old)
            for i, line in enumerate(lines):
                line, n = pattern.subn(
                    lambda mm, new=new, f=fname_part: f"{f}:{mm.group(1)}{new}", line
                )
                if not n:
                    continue
                total += n
                lines[i] = adjacency_re(old).sub(rf":{new}\1", line)
        if len(lines) != len(original):
            raise SystemExit(f"FAIL {ledger}: rewrite changed the line count")
        path.write_text("\n".join(lines), encoding="utf-8")
    return total


def leftover_spellings(moves):
    """Occurrences of a moved line number under a DIFFERENT path spelling
    than the row's own -- `parry.rs:4623` where the row's key was
    `crates/.../parry.rs:4623`. Reported, never rewritten: a bare basename
    can belong to another file of the same name, and this tool does not
    guess which."""
    out = []
    for ledger, fname_part, old, _new, _fn in moves:
        text = (ROOT / ledger).read_text(encoding="utf-8")
        base = fname_part.rsplit("/", 1)[-1]
        if base == fname_part:
            continue
        hits = len(cite_re(base, old).findall(text))
        if hits:
            out.append((ledger, base, old, hits))
    return out


def main(argv):
    apply = "--apply" in argv[1:]
    mod = load_reconcile()
    moves, skipped = plan(mod)
    for ledger, fname_part, old, new, name in moves:
        print(f"  {ledger}: {fname_part}:{old} -> :{new}  (fn {name})")
    for cite, why in skipped:
        print(f"  SKIP {cite} -- {why}", file=sys.stderr)
    if not apply:
        print(f"{len(moves)} would move, {len(skipped)} need a human "
              f"(re-run with --apply to write)")
        return 0
    total = apply_moves(moves)
    print(f"repointed {total} citation(s) across {len(moves)} row(s)")
    for ledger, base, old, hits in leftover_spellings(moves):
        print(f"  LEFT {ledger}: {hits} occurrence(s) of `{base}:{old}` under a "
              f"shorter path spelling -- check by hand", file=sys.stderr)
    if skipped:
        print(f"{len(skipped)} citation(s) skipped -- resolve those by hand", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
