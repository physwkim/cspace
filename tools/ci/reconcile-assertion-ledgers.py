#!/usr/bin/env python3
# Usage: tools/ci/reconcile-assertion-ledgers.py [--emit-orphans] [--emit-unresolved]
#
# Assertion-discrimination sweep instrument (see
# doc/assertion-discrimination-census.md, and the p3-acm ledger's "Round 14
# -- closing audit" section): partitions every site
# `count-coarse-assertions.py` finds into MATCHED (some ledger row accounts
# for it) or ORPHAN (no ledger row does), so that partition is reproducible
# from a clean checkout instead of resting on an uncommitted scratchpad
# script and a hand-written table.
#
# This does NOT re-verify any ledger's discrimination verdicts. It answers
# one narrower question: does every coarse-assertion site the scanner finds
# have a ledger row, or a documented reason for having none?
#
# Three ways a ledger row can account for a scanner site:
#
#   1. Exact match -- the row's first-column `file:line` is exactly a
#      scanner site.
#   2. Unique nearby match -- the row's citation is within
#      NEARBY_WINDOW lines of exactly one scanner site in the same file.
#      Small, silent line drift (a comment added a line above, say) is
#      common enough that treating every one of these as a human question
#      would bury the real gaps. A window match against MULTIPLE
#      candidate sites is deliberately NOT auto-resolved -- see
#      `ambiguous_window` below.
#   3. A vetted equivalence in `assertion-ledger-equivalences.json` --
#      for the two shapes a window can't cover: (a) a ledger that cites a
#      guard's own production line instead of the assert that exercises
#      it (moveit-collision's `tools.rs:68 (x)/(y)/(z)` vs `tools.rs:259,
#      271,283`), confirmed by reading that ledger's own prose explaining
#      the correspondence; (b) line drift larger than the window,
#      confirmed by matching the row's own named test function to its
#      current line in the source, not by nearest-line proximity (nearest-
#      line proximity picked the WRONG site at least once while this
#      instrument's equivalences were being derived -- see the JSON
#      file's own comments). Every entry names the evidence; there is no
#      "trust me" entry.
#
# Anything a ledger cites that resolves under none of the three is left in
# `unresolved_citations` -- reported, never silently dropped and never
# silently guessed at. A best-effort heuristic (`classify_citation`) reads
# the actual line the ledger cites and tags *why* automated matching
# failed (looks like a guard/`?`-propagation line, looks like a comment,
# or no clue at all), strictly to help a human router, never to auto-
# resolve.
import json
import re
import subprocess
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
SCANNER = REPO_ROOT / "tools" / "ci" / "count-coarse-assertions.py"
EQUIVALENCES_FILE = REPO_ROOT / "tools" / "ci" / "assertion-ledger-equivalences.json"
LEDGERS = [
    "doc/assertion-discrimination-ledger-p1-fixtures.md",
    "doc/assertion-discrimination-ledger-p1-robotmodel.md",
    "doc/assertion-discrimination-ledger-p3-acm.md",
    "doc/assertion-discrimination-ledger-p9-ros.md",
    "doc/assertion-discrimination-ledger-pilz.md",
]
NEARBY_WINDOW = 5


# Trailing text is intentionally permitted between the line number(s) and the
# closing `|` (e.g. "tools.rs:68 (x)") -- some ledgers' first column disambiguates
# a fanned-out guard-line citation with an "(x)/(y)/(z)" suffix, and a regex
# that required the pipe to follow immediately would silently drop those rows
# from parsing altogether (worse than reporting them unresolved).
FIRST_COL_RE = re.compile(
    r"^\|\s*`?((?:[\w./-]+/)?[\w.-]+\.rs):(\d+(?:\s*,\s*\d+)*)\s*`?[^|]*\|"
)


def run_scanner():
    """Live scanner sites, excluding helper_body. Never reads a cached file --
    this is the whole point of "reproducible from a clean checkout"."""
    out = subprocess.run(
        [sys.executable, str(SCANNER), "crates/", "ros/", "tools/"],
        cwd=REPO_ROOT,
        capture_output=True,
        text=True,
        check=True,
    ).stdout
    sites = {}
    basenames = {}
    for line in out.splitlines():
        if not line.strip():
            continue
        path, lineno, kind, rest = line.split(":", 3)
        scope = rest.split(":", 1)[0]
        if scope == "helper_body":
            continue
        lineno = int(lineno)
        sites[(path, lineno)] = kind
        basenames.setdefault((path.rsplit("/", 1)[-1], lineno), []).append(path)
    return sites, basenames


def parse_ledger_citations(ledger_rel):
    """(cited_file, cited_line, raw_row) for every genuine table-row site
    citation in one ledger. Deliberately does NOT expand hyphen ranges
    (`state.rs:369-513`): every ledger sampled cites those as prose
    references to a whole code region, not a per-site table row -- expanding
    them manufactured ~145 phantom citations from one row the first time
    this was tried by hand. Only comma-separated first-column citations are
    genuine multi-site rows."""
    citations = []
    text = (REPO_ROOT / ledger_rel).read_text(encoding="utf-8")
    for row in text.splitlines():
        m = FIRST_COL_RE.match(row)
        if not m:
            continue
        fname_part, nums_part = m.group(1), m.group(2)
        for n in nums_part.split(","):
            citations.append((fname_part, int(n.strip()), row.strip()[:160]))
    return citations


def resolve(fname_part, lineno, sites, basenames):
    """Exact match, else a UNIQUE match within +/- NEARBY_WINDOW lines in the
    same file. Returns (resolved_site_or_None, status) where status is one
    of "exact", "window", "ambiguous-exact", "ambiguous-window", "none"."""
    if "/" in fname_part:
        exact = [p for (p, ln) in sites if ln == lineno and p.endswith(fname_part)]
    else:
        exact = basenames.get((fname_part, lineno), [])
    exact = sorted(set(exact))
    if len(exact) == 1:
        return (exact[0], lineno), "exact"
    if len(exact) > 1:
        return None, "ambiguous-exact"

    candidates = set()
    for dl in range(1, NEARBY_WINDOW + 1):
        for cand_line in (lineno - dl, lineno + dl):
            if "/" in fname_part:
                hits = [p for (p, ln) in sites if ln == cand_line and p.endswith(fname_part)]
            else:
                hits = basenames.get((fname_part, cand_line), [])
            for h in hits:
                candidates.add((h, cand_line))
    if len(candidates) == 1:
        return next(iter(candidates)), "window"
    if len(candidates) > 1:
        return None, "ambiguous-window"
    return None, "none"


def classify_citation(fname_part, lineno):
    """Best-effort, report-only heuristic for why a citation didn't
    resolve -- read the actual line the ledger points at and say what shape
    it is. Never used to auto-match; only to help a human triage
    `unresolved_citations` faster."""
    candidates = list(REPO_ROOT.glob(f"**/{fname_part}"))
    candidates = [c for c in candidates if "target" not in c.parts]
    for c in candidates:
        try:
            lines = c.read_text(encoding="utf-8", errors="replace").splitlines()
        except OSError:
            continue
        if 0 < lineno <= len(lines):
            src = lines[lineno - 1].strip()
            if src.startswith("//") or src.startswith("///"):
                return "cites a comment line, not an assertion"
            if re.search(r"\)\?\s*;?\s*$", src) or re.search(r"\)\?\s*[,;)]", src):
                return "cites a `?`-propagation guard line, not an assertion -- outside the scanner's grammar by design"
            return f"no scanner match; source there reads: {src[:80]!r}"
    return "cited file not found under this root"


def load_equivalences():
    if not EQUIVALENCES_FILE.exists():
        return {}
    data = json.loads(EQUIVALENCES_FILE.read_text(encoding="utf-8"))
    out = {}
    for entry in data["equivalences"]:
        key = (entry["ledger"], entry["cited_file"], entry["cited_line"])
        out[key] = entry
    return out


def main(argv):
    emit_orphans = "--emit-orphans" in argv
    emit_unresolved = "--emit-unresolved" in argv

    sites, basenames = run_scanner()
    equivalences = load_equivalences()

    matched_sites = set()
    match_notes = []  # (ledger, cited_file, cited_line, resolved_site, how)
    unresolved = []  # (ledger, cited_file, cited_line, raw_row, why)
    non_scope = []  # (ledger, cited_file, cited_line, reason)

    for ledger in LEDGERS:
        for fname_part, lineno, raw in parse_ledger_citations(ledger):
            resolved, status = resolve(fname_part, lineno, sites, basenames)
            if resolved is not None:
                matched_sites.add(resolved)
                match_notes.append((ledger, fname_part, lineno, resolved, status))
                continue

            eq = equivalences.get((ledger, fname_part, lineno))
            if eq is not None:
                if eq["resolution"] == "non_scope":
                    non_scope.append((ledger, fname_part, lineno, eq["reason"]))
                elif eq["resolution"] == "matches":
                    # A guard-line citation can fan out to more than one
                    # assert site (moveit-collision's tools.rs:68 case: one
                    # guard, three axis-isolating tests) -- accept either a
                    # single [file, line] pair or a list of them.
                    raw_sites = eq["scanner_sites"] if "scanner_sites" in eq else [
                        [eq["scanner_file"], eq["scanner_line"]]
                    ]
                    for sf, sl in raw_sites:
                        resolved = (sf, sl)
                        matched_sites.add(resolved)
                        match_notes.append((ledger, fname_part, lineno, resolved, "equivalence: " + eq["reason"]))
                else:
                    raise ValueError(f"unknown equivalence resolution {eq['resolution']!r}")
                continue

            why = classify_citation(fname_part, lineno)
            unresolved.append((ledger, fname_part, lineno, raw, f"{status}; {why}"))

    all_scanner_sites = set(sites)
    orphans = sorted(all_scanner_sites - matched_sites)

    total = len(all_scanner_sites)
    matched = len(matched_sites)
    orphan_count = len(orphans)

    print(f"scanner sites (excl. helper_body): {total}")
    print(f"matched (some ledger row accounts for the site): {matched}")
    print(f"orphans (no ledger row accounts for the site):    {orphan_count}")
    print(f"check: matched + orphans == scanner sites -> {matched + orphan_count == total}")
    print()
    print(f"ledger citations resolved via vetted equivalence: "
          f"{sum(1 for n in match_notes if n[4].startswith('equivalence'))}")
    print(f"ledger citations explained as non-scanner-scope (not gaps): {len(non_scope)}")
    for ledger, f, ln, reason in non_scope:
        print(f"  [{Path(ledger).stem}] {f}:{ln} -- {reason}")
    print(f"ledger citations still unresolved (reported, not guessed): {len(unresolved)}")
    for ledger, f, ln, raw, why in unresolved:
        print(f"  [{Path(ledger).stem}] {f}:{ln} -- {why}")

    if emit_orphans:
        print()
        print(f"--- {orphan_count} orphans (file:line:kind) ---")
        for path, line in orphans:
            print(f"{path}:{line}:{sites[(path, line)]}")

    if emit_unresolved:
        print()
        print(f"--- {len(unresolved)} unresolved ledger citations ---")
        for ledger, f, ln, raw, why in unresolved:
            print(f"[{Path(ledger).stem}] {f}:{ln} :: {why}")

    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
