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

Column 3 is the subject column in every ledger layout in this repository.
That is measured, not assumed: over the 1087 citations that resolve exactly
today, 272 have a bare identifier in column 3 that names a function in the
cited file, and in 272 of 272 that function is the site's own enclosing
function -- zero disagreements. (The other rows name the production function
under test, or a prose phrase like `(same test)`; both are left for a human,
never guessed at.)

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

# A whole cell that is one snake_case identifier, optionally backticked.
# Anything else in column 3 -- `(same test)`, a prose phrase, two names --
# yields no relocation key, which is the intended outcome: this tool has
# nothing content-grounded to move that row to.
IDENT_CELL_RE = re.compile(r"^`?([a-z_][a-z0-9_]{5,})`?$")
FN_DEF_RE = re.compile(r"^\s*(?:pub\s+)?(?:async\s+)?fn\s+([A-Za-z0-9_]+)")


def load_reconcile():
    spec = importlib.util.spec_from_file_location("reconcile", RECONCILE)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


_fn_cache = {}


def fn_defs(rel_path):
    """[(line, name)] for every `fn` in a source file, in file order."""
    if rel_path not in _fn_cache:
        lines = (ROOT / rel_path).read_text(encoding="utf-8").splitlines()
        _fn_cache[rel_path] = [
            (i + 1, m.group(1)) for i, line in enumerate(lines) if (m := FN_DEF_RE.match(line))
        ]
    return _fn_cache[rel_path]


def fn_span(rel_path, name):
    """[(first_line, last_line)] for each definition of `name`. The span ends
    at the next `fn` rather than at a matching brace: a nested `fn` inside a
    test body would end the span early, and no ledger's subject function has
    one. Two definitions of the same name (two `mod tests` in one file) make
    the key ambiguous and the row is skipped."""
    defs = fn_defs(rel_path)
    out = []
    for i, (line, n) in enumerate(defs):
        if n == name:
            end = defs[i + 1][0] - 1 if i + 1 < len(defs) else 10**9
            out.append((line, end))
    return out


def row_cells(raw_row):
    return raw_row.split("|")


def subject_fn(raw_row):
    cells = row_cells(raw_row)
    if len(cells) < 5:
        return None
    m = IDENT_CELL_RE.match(cells[3].strip())
    return m.group(1) if m else None


def row_kind(raw_row):
    cells = row_cells(raw_row)
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
    orphans = set(result["orphans"])
    first_population = set(result["orphans_first"])
    moves, skipped = [], []
    for ledger, fname_part, lineno, raw, _status, _cat, _detail in result["unresolved"]:
        cite = f"{ledger} -> {fname_part}:{lineno}"
        name = subject_fn(raw)
        if name is None:
            skipped.append((cite, "column 3 is not a bare function name"))
            continue
        files = candidate_files(mod, fname_part, sites)
        if len(files) != 1:
            skipped.append((cite, f"{len(files)} source file(s) match this citation"))
            continue
        path = files[0]
        spans = fn_span(path, name)
        if len(spans) != 1:
            skipped.append((cite, f"`fn {name}` is defined {len(spans)}x in {path}"))
            continue
        lo, hi = spans[0]
        inside = sorted(
            line for (p, line) in orphans if p == path and lo <= line <= hi
        )
        if not inside:
            skipped.append((cite, f"`fn {name}` holds no orphan scanner site"))
            continue
        first = [line for line in inside if (path, line) in first_population]
        want = row_kind(raw)
        same_kind = [line for line in inside if sites[(path, line)] == want]
        if len(first) == 1:
            new = first[0]
        elif len(same_kind) == 1:
            new = same_kind[0]
        elif len(inside) == 1:
            new = inside[0]
        else:
            skipped.append(
                (cite, f"`fn {name}` holds {len(inside)} orphan sites {inside} "
                       f"({len(first)} in the first population, {len(same_kind)} of "
                       f"kind {want!r}) -- ambiguous")
            )
            continue
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


def apply_moves(moves):
    """Rewrite each moved citation EVERYWHERE in its own ledger, not only in
    the row's first column. A ledger's prose cites the same site again --
    `parry.rs:4623` appears both as a row key and inside that ledger's
    "0 misverdicts" audit paragraph -- and those prose citations are in
    `check-citation-drift.py`'s corpus, so a first-column-only rewrite trades
    one gate's failure for another's. A document-wide rewrite is safe here
    precisely because the plan is 1:1: if two rows cited the same old line
    they would both be unresolved and could want different new lines, so
    that case refuses to write rather than pick one."""
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
        text = original = path.read_text(encoding="utf-8")
        for fname_part, old, new in items:
            text, n = cite_re(fname_part, old).subn(
                lambda mm, new=new, f=fname_part: f"{f}:{mm.group(1)}{new}", text
            )
            total += n
        if text.count("\n") != original.count("\n"):
            raise SystemExit(f"FAIL {ledger}: rewrite changed the line count")
        path.write_text(text, encoding="utf-8")
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
